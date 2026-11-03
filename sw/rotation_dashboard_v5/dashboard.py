"""V4 preview and controls bound to the selected algorithm1080_nn_r1 R14 context."""
from __future__ import annotations

import argparse
from collections import deque
import contextlib
import ctypes
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import queue
import re
import sys
import time
import traceback
import uuid

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
LEGACY = HERE.parent / 'rotation_console_v1'
sys.path.insert(0, str(LEGACY))

import psutil
from PySide6.QtCore import QObject, QThread, QTimer, Signal, Slot, Qt
from PySide6.QtWidgets import QApplication, QMessageBox, QFileDialog, QInputDialog
from rotation_console import (RotationConsole, DEFAULT_PROFILE,
                              parse_command, write_json)
from roll_estimator import estimate_roll
from dashboard_view import DashboardWindow
from preview_stream import PreviewStream
from telemetry_client import read_observations, read_a5_state
from monitor_model import RateTracker, VideoProgressTracker
from auto_correction import (AutoCorrectionError, AutoCorrectionPolicy, apply_proposal,
                             apply_manual_angle)
from interactive_control import LiveBoardController


def utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def assert_no_legacy_console() -> dict[str, list[int]]:
    """Block only positively identified instances of this legacy console."""
    wanted = os.path.normcase(str((LEGACY/'rotation_console.py').resolve()))
    conflicts = []
    unknown_pids = set()
    for proc in psutil.process_iter(['pid', 'name']):
        if proc.pid == os.getpid() or 'python' not in (proc.info.get('name') or '').lower():
            continue
        try:
            argv = proc.cmdline()
        except psutil.NoSuchProcess:
            continue
        except psutil.AccessDenied:
            unknown_pids.add(proc.pid)
            continue
        if not argv:
            unknown_pids.add(proc.pid)
            continue

        matched = False
        for arg in argv:
            if not arg or arg.replace('\\', '/').rsplit('/', 1)[-1].lower() != 'rotation_console.py':
                continue
            script = Path(arg)
            if not script.is_absolute():
                try:
                    process_cwd = proc.cwd()
                except psutil.NoSuchProcess:
                    break
                except psutil.AccessDenied:
                    unknown_pids.add(proc.pid)
                    continue
                if not process_cwd:
                    unknown_pids.add(proc.pid)
                    continue
                script = Path(process_cwd) / script
            if os.path.normcase(str(script.resolve())) == wanted:
                matched = True
                break
        if matched:
            conflicts.append(proc.pid)
    if conflicts:
        raise RuntimeError('请先在旧命令窗口输入quit并回车，再点击连接控制板。旧控制台PID：'
                           + ', '.join(map(str, conflicts)))
    return {'unknown_pids': sorted(unknown_pids)}


class SharedPreviewConsole(RotationConsole):
    """Keep the original correction transaction; replace only frame acquisition."""
    def __init__(self, controller, run_dir, device, preview):
        super().__init__(controller, run_dir, device)
        self.preview = preview

    def _capture_estimate(self, folder, label, record):
        self.controller.require_unchanged()
        batch = self.preview.collect(folder/label)
        state = self.controller.require_unchanged()
        estimate = estimate_roll(batch.frames)
        write_json(folder/(label+'_estimate.json'), estimate.to_dict())
        record.setdefault('measurements', []).append({
            'capture_record': str(batch.record_path),
            'capture_record_sha256': sha(batch.record_path),
            'estimate': estimate.to_dict(),
            'observed_configuration': state['algorithmConfig'],
            'source': 'shared_actual_hdmi_preview',
        })
        return estimate


class BoardObservationConnection:
    """A validated serial observation endpoint without parameter-write ownership."""

    def __init__(self, port):
        self.port = port
        self.current_angle = None
        self.last_applied = None
        tools = (HERE.parents[1] / 'wanchengceshi' / 'algorithm_v04_1080p_upgrade'
                 / 'algorithm1080_rejoin_r8' / 'tools')
        self.diagnostic = read_a5_state(port, tools)

    def crop_state(self):
        return {'known': False, 'enabled': None, 'zoom': None}


class R14BoundController:
    """R14-selected acceptance/session owner for every V4 geometry write."""
    def __init__(self, acceptance: Path, session: Path, profile: Path, port: str, work_dir: Path, notify):
        self.acceptance_path, self.session_path = Path(acceptance).resolve(), Path(session).resolve()
        self.profile_path, self.port = Path(profile).resolve(), str(port)
        self.work_dir, self.notify = Path(work_dir), notify
        self.current_angle = None
        self.last_applied = None
        self.crop_enabled = None
        self.crop_applied = None
        self._prepared_config = None
        self._validate_context()
        state = self._fresh_state()
        self.last_applied = self._ack_from(state)

    @property
    def candidate_tools(self):
        return HERE.parents[1] / 'wanchengceshi' / 'algorithm_v04_1080p_upgrade' / 'algorithm1080_nn_r1' / 'tools'

    def _validate_context(self):
        from auto_correction import _validate_r14_program_binding
        return _validate_r14_program_binding(self.acceptance_path, self.session_path)

    @staticmethod
    def _ack_from(state):
        config = state.get('algorithmConfig') or {}
        return {key: int(config[key]) for key in ('receivedSequence', 'appliedSequence', 'activeVersion')}

    def _fresh_state(self):
        self._validate_context()
        return read_a5_state(self.port, self.candidate_tools.parent.parent / 'algorithm1080_rejoin_r8' / 'tools')

    def require_unchanged(self):
        if not self.last_applied:
            self._invalidate_ownership()
            raise RuntimeError('无可比较的 R14 A5 基线，请重新连接控制板。')
        try:
            state = self._fresh_state()
        except Exception:
            self._invalidate_ownership()
            raise
        ack = self._ack_from(state)
        if any(ack[key] != self.last_applied[key] for key in self.last_applied):
            self._invalidate_ownership()
            raise RuntimeError('R14 A5 基线变化；外部写入或复位使本机配置所有权未知。')
        return state

    def _invalidate_ownership(self):
        self.current_angle = None
        self.last_applied = None
        self.crop_enabled = None
        self.crop_applied = None
        self._prepared_config = None

    def record_auto_applied(self, angle, ack, config_dir, crop_report):
        self.current_angle = float(angle)
        self.last_applied = {key: int(ack[key]) for key in ('receivedSequence', 'appliedSequence', 'activeVersion')}
        self._prepared_config = Path(config_dir)
        self.crop_enabled = bool(crop_report.get('cropEnabled', False))
        self.crop_applied = dict(crop_report) if self.crop_enabled else None

    def apply_angle(self, angle):
        result = apply_manual_angle(self, float(angle), self.acceptance_path, self.session_path,
            self.profile_path, self.work_dir / 'manual_geometry', crop=bool(self.crop_enabled),
            progress=self.notify, allow_unknown_initial=True)
        return result

    def crop_state(self):
        known = self.current_angle is not None and self.last_applied is not None and self.crop_enabled is not None
        return {'known': known, 'enabled': self.crop_enabled if known else None,
                'zoom': (self.crop_applied or {}).get('zoom', 1.0) if known else None,
                'report': str(self._prepared_config / 'crop_report.json')
                          if known and self._prepared_config else None}

    def set_crop(self, enabled):
        if self.current_angle is None:
            raise RuntimeError('请先手动设置角度建立 R14 ACK，再更改裁剪。')
        prior = self.crop_enabled
        try:
            result = apply_manual_angle(self, float(self.current_angle), self.acceptance_path,
                self.session_path, self.profile_path, self.work_dir / 'manual_geometry',
                crop=bool(enabled), progress=self.notify)
            return result
        except BaseException:
            if self.current_angle is None:
                self.crop_enabled = None
                self.crop_applied = None
                self._prepared_config = None
            else:
                self.crop_enabled = prior
            raise


class LogWriter(io.TextIOBase):
    def __init__(self, notify, stream):
        self.notify, self.stream, self.buffer = notify, stream, ''

    def write(self, text):
        self.stream.write(text)
        self.stream.flush()
        self.buffer += text
        while '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\n', 1)
            if line.strip():
                self.notify(line.strip())
        return len(text)

    def flush(self):
        if self.buffer.strip():
            self.notify(self.buffer.strip())
        self.buffer = ''
        self.stream.flush()


class ControlWorker(QThread):
    message = Signal(object)

    def __init__(self, preview, run_dir, args):
        super().__init__()
        self.preview, self.run_dir, self.args = preview, run_dir, args
        self.requests = queue.Queue(maxsize=1)
        self.controller = None
        self.console = None
        self.monitor_a8_unsupported = False
        self.auto_enabled = False

    def submit(self, kind, value=None):
        self.requests.put_nowait((kind, value))

    def notify(self, text):
        uploaded = re.search(r'^uploaded (\d+)/(\d+) bytes', str(text))
        if uploaded:
            done, total = map(int, uploaded.groups())
            text = f'正在加载 CNN：{100 * done / total:.1f}%'
        elif str(text).startswith('BEGIN acknowledged'):
            text = '开始加载 CNN 模型，首次需要几分钟'
        self.message.emit({'type': 'log', 'text': text})

    def state(self):
        angle = self.controller.current_angle if self.controller else None
        write_ready = (isinstance(self.controller, (R14BoundController, LiveBoardController))
                       and bool(self.controller.last_applied)
                       and not getattr(self.controller, 'video_stalled', False))
        return {'connected': self.controller is not None,
                'control_ready': write_ready,
                'needs_reconnect': self.controller is not None and not write_ready,
                'live_control': isinstance(self.controller, LiveBoardController),
                'angle': math.remainder(angle, 360.0) if angle is not None else None,
                'ack': self.controller.last_applied.copy() if self.controller and self.controller.last_applied else None,
                'crop': self.controller.crop_state() if self.controller else {'known': False, 'enabled': None, 'zoom': None}}

    def execute(self, kind, value):
        if kind == 'monitor':
            if self.controller is None:
                return {'freshness': 'unknown', 'detail': '请先连接控制板'}
            root = HERE.parents[1] / 'wanchengceshi' / 'algorithm_v04_1080p_upgrade' / 'algorithm1080_nn_r1' / 'tools'
            return read_observations(self.controller.port, root, include_a8=not self.monitor_a8_unsupported,
                                     include_a5=True)
        if kind == 'nn_action':
            if isinstance(self.controller, LiveBoardController) and dict(value).get('kind') == 'prepare':
                result = self.controller.prepare_cnn(progress=self.notify)
                return ('CNN 已准备并启用', '等待网络产生新预测后，可开启 CNN 自动校正。', 'success')
            if self.controller is None or not getattr(self, 'monitor_context', None):
                raise RuntimeError('连接控制板并选择匹配的候选验收与编程会话后，才能执行模型操作。')
            action = dict(value)
            tools_dir = HERE.parents[1] / 'wanchengceshi' / 'algorithm_v04_1080p_upgrade' / 'algorithm1080_nn_r1' / 'tools'
            from auto_correction import _validate_r14_program_binding
            _validate_r14_program_binding(*self.monitor_context)
            action_output = self.run_dir / 'nn_actions' / uuid.uuid4().hex
            action_output.parent.mkdir(exist_ok=True)
            argv = [sys.executable, '-B', str(tools_dir / 'nn_board_observe.py'),
                    '--port', self.controller.port, '--acceptance', str(self.monitor_context[0]),
                    '--session', str(self.monitor_context[1]), '--output', str(action_output)]
            if action['kind'] == 'load': argv.append('--upload-weights')
            elif action['kind'] == 'enable': argv.append('--enable-observation' if action['value'] else '--disable-observation')
            elif action['kind'] == 'k':
                argv += ['--set-k', '--fx', str(action['values'][0]), '--fy', str(action['values'][1]),
                         '--cx', str(action['values'][2]), '--cy', str(action['values'][3])]
            else: raise ValueError('Unknown model action')
            argv += ['--status-count', '1', '--status-timeout', '4']
            import subprocess
            command_record = {'argv': argv, 'startedUtc': utc(), 'action': action}
            try:
                completed = subprocess.run(argv, capture_output=True, text=True, timeout=1800,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            except subprocess.TimeoutExpired as exc:
                action_output.mkdir(parents=True, exist_ok=True)
                (action_output / 'stdout.log').write_text(str(exc.stdout or ''), encoding='utf-8')
                (action_output / 'stderr.log').write_text(str(exc.stderr or '') + '\nTIMEOUT', encoding='utf-8')
                command_record.update({'endedUtc': utc(), 'status': 'timeout'})
                (action_output / 'command.json').write_text(json.dumps(command_record, indent=2), encoding='utf-8')
                raise RuntimeError('模型操作超时；输出已保存在 ' + str(action_output)) from exc
            action_output.mkdir(parents=True, exist_ok=True)
            (action_output / 'stdout.log').write_text(completed.stdout or '', encoding='utf-8')
            (action_output / 'stderr.log').write_text(completed.stderr or '', encoding='utf-8')
            command_record.update({'endedUtc': utc(), 'returnCode': completed.returncode})
            (action_output / 'command.json').write_text(json.dumps(command_record, indent=2), encoding='utf-8')
            if completed.returncode:
                raise RuntimeError(((completed.stderr or completed.stdout or '模型操作失败').strip()[-2000:]) + f'；记录：{action_output}')
            return ('NN_ACTION', completed.stdout.strip())
        if kind == 'auto_apply':
            if not getattr(self, 'auto_enabled', False):
                raise RuntimeError('CNN 自动校正已关闭；取消本次预测更新。')
            if not isinstance(self.controller, (R14BoundController, LiveBoardController)):
                raise RuntimeError('CNN 自动校正缺少连接或匹配的候选上下文。')
            if isinstance(self.controller, LiveBoardController):
                acceptance, session = self.controller.acceptance_path, self.controller.session_path
            else:
                acceptance, session = self.monitor_context
            crop = bool((self.controller.crop_state() or {}).get('enabled'))
            result = apply_proposal(self.controller, value, acceptance, session,
                self.args.profile, self.run_dir / 'cnn_auto_correction', crop=crop,
                progress=self.notify, cancel_check=lambda: bool(getattr(self, 'auto_enabled', False)),
                binding_validator=getattr(self.controller, 'validate_auto_context',
                                          getattr(self.controller, 'binding_validator', None)))
            return result
        if kind == 'preview':
            self.preview.stop()
            self.preview.start()
            return ('已启动预览', '等待采集卡返回实际画面；无需打开Windows相机。', 'info')
        if self.args.offline_demo:
            raise RuntimeError('当前为离线界面演示，不连接串口或操作板卡。')
        assert_no_legacy_console()
        if kind == 'connect':
            port = str(value).strip().upper()
            if not re.fullmatch(r'COM[1-9][0-9]*', port):
                raise ValueError('请输入有效UART端口，例如COM14。')
            if not getattr(self, 'monitor_context', None):
                self.controller = None
                self.console = None
                controller = LiveBoardController(port, self.args.profile,
                    self.run_dir / ('board_' + uuid.uuid4().hex), self.notify)
                controller.connect()
                self.controller = controller
                self.console = SharedPreviewConsole(controller, self.run_dir, self.args.device, self.preview)
                return ('控制板已连接', '可输入角度并点击应用；CNN 先点击“准备并启用”。', 'success')
            self.notify('正在核对所选 R14 候选验收、编程会话并读取 A5 基线……')
            self.controller = None
            self.console = None
            acceptance, session = self.monitor_context
            controller = R14BoundController(acceptance, session, self.args.profile,
                port, self.run_dir/'control_r14', notify=self.notify)
            console = SharedPreviewConsole(controller, self.run_dir, self.args.device, self.preview)
            console.numeric(0, 'startup_real0')
            self.controller, self.console = controller, console
            write_json(self.run_dir/'connection.json', {'connectedUtc': utc(), 'session': str(session),
                'acceptance': str(acceptance), 'profile': str(self.args.profile),
                'candidate': 'algorithm1080_nn_r1', 'port': port,
                'baseline': controller.last_applied, 'state': self.state()})
            return ('R14连接成功 · A5基线及0° ACK已确认', '当前会话绑定所选 algorithm1080_nn_r1 候选；CNN 自动校正仍需用户显式开启。', 'success')
        if self.console is None or self.controller is None:
            raise RuntimeError('请先连接控制板。')
        if kind == 'angle':
            command, angle = parse_command(str(value))
            if command != 'angle':
                raise ValueError('角度框请输入有限数字，例如20、-15或13.37。')
            self.console.numeric(float(angle))
            return ('角度已实际生效', f'板卡确认 {angle:+g}°，真实镜头校正保持开启。', 'success')
        if kind == 'crop':
            result = self.controller.set_crop(bool(value))
            crop = self.controller.crop_state()
            write_json(self.run_dir/('crop_'+uuid.uuid4().hex+'.json'),
                       {'completedUtc': utc(), 'result': result, 'state': self.state()})
            if crop['enabled']:
                return ('去边参数已由板卡确认',
                        f"等比放大 {crop['zoom']:.3f}×，保留有效视野；实际画质和处理率以采集结果为准。", 'success')
            return ('已恢复完整视野', '未去边时保留完整视野；边缘外观取决于当前FPGA配置。', 'info')
        if kind == 'correct':
            if self.controller.current_angle is None:
                self.controller.apply_angle(0)
            if self.preview.latest() is None:
                raise RuntimeError('没有新鲜预览画面，请先恢复视频或关闭占用采集卡的相机软件。')
            prior = {p for p in self.run_dir.glob('correct_*') if p.is_dir()}
            self.console.correct()
            folders = {p for p in self.run_dir.glob('correct_*') if p.is_dir()}-prior
            if len(folders) != 1:
                raise RuntimeError('缺少唯一的自动摆正操作记录，不能确认结果。')
            result = json.loads((folders.pop()/'result.json').read_text(encoding='utf-8'))
            status = result['status']
            if status == 'CORRECTED_VISUALLY_CONFIRMED':
                return ('自动摆正已复测确认', f"剩余倾斜 {result['residual_degrees']:+.3f}°；以棋盘或场景线为参照。", 'success')
            if status == 'ALREADY_LEVEL':
                return ('画面已经接近水平', f"实测偏转 {result['residual_degrees']:+.3f}°，保持当前角度。", 'success')
            if status == 'REFUSED_NO_RELIABLE_REFERENCE':
                return ('参照不足，保持原角度', '请提供水平棋盘格或分布充分的横竖线，并保持场景静止。', 'warning')
            rollback = result.get('rollback', {})
            if rollback.get('acknowledged'):
                return ('自动摆正复测未通过', '已经回读确认恢复到操作前的角度。', 'warning')
            return ('自动摆正未完成', '请查看操作记录；角度未知时先输入数字重新建立基准。状态：'+status, 'error')
        raise ValueError('Unknown operation')

    def run(self):
        with (self.run_dir/'activity.log').open('a', encoding='utf-8', buffering=1) as log:
            while True:
                kind, value = self.requests.get()
                if kind == 'shutdown':
                    try:
                        self.preview.stop()
                        status = self.preview.status()
                        if status['process_alive'] or status['reader_alive']:
                            raise RuntimeError('本程序的视频进程尚未完全退出，请再次关闭以重试释放。')
                        self.message.emit({'type': 'shutdown', 'state': self.state()})
                    except Exception as exc:
                        log.write(traceback.format_exc()+'\n')
                        self.message.emit({'type': 'shutdown_failed', 'error': str(exc)})
                        continue
                    return
                writer = LogWriter(self.notify, log)
                result = None
                try:
                    with contextlib.redirect_stdout(writer):
                        result = self.execute(kind, value)
                        notice = (('监测完成', '', 'info') if kind == 'monitor' else
                                  ('CNN 自动校正已确认', str(result.get('applied', {})), 'success') if kind == 'auto_apply' else result)
                except Exception as exc:
                    log.write(traceback.format_exc()+'\n')
                    self.notify(str(exc))
                    error = str(exc)
                    if 'No fresh A5 connection ACK' in error or 'ownership' in error.lower():
                        error = '上次参数确认已丢失，请点击“重新连接”。'
                    elif 'No fresh sequence/version/count ACK' in error:
                        error = '板卡未确认角度；视频处理可能已停住，请先恢复板卡再重新连接。'
                    elif kind == 'nn_action' and 'not acknowledged' in error:
                        error = '模型加载命令未完成，已停止；请勿反复加载，先恢复板卡运行。'
                    elif 'changed' in error.lower() and 'source' in error.lower():
                        error = '控制程序已更新，请关闭窗口后重新打开。'
                    notice = ('操作未完成', error, 'error')
                    if kind == 'auto_apply' and getattr(self, 'auto_policy', None):
                        uncertain = bool(getattr(exc, 'uncertain', False))
                        ownership_lost = bool(getattr(exc, 'ownership_lost', False))
                        if uncertain:
                            self.auto_policy.failed()
                        else:
                            self.auto_policy.refused()
                        result = {'uncertain': uncertain, 'ownership_lost': ownership_lost, 'error': str(exc)}
                finally:
                    writer.flush()
                self.message.emit({'type': 'done', 'kind': kind, 'notice': notice, 'result': result if kind in ('monitor', 'auto_apply') else None, 'state': self.state()})


class Coordinator(QObject):
    def __init__(self, window, preview, run_dir, args):
        super().__init__(window)
        self.window, self.preview, self.run_dir = window, preview, run_dir
        self.busy = False
        self.pending_close = False
        self.pending_angle = None
        self.pending_crop = None
        self.pending_action = None
        self.active_kind = None
        self.active_value = None
        self.shutting_down = False
        self.control = {'connected': False, 'angle': None, 'ack': None}
        self.state_text = '请连接控制板'
        self.busy_since = 0.0
        self.latest_sequence = None
        self.frame_submissions = 0
        self.host_submission_skipped_frames = 0
        self.paint_times = deque(maxlen=180)
        self.qt_painted_frames = 0
        self.qt_paint_skipped_frames = 0
        self.last_qt_painted_identity = None
        self.last_status_write = 0.0
        self.snapshot_saved = False
        self.monitor_state = {'freshness': 'unknown', 'detail': '尚无设备监测快照', 'cnn': {}, 'gamma': {}}
        self.monitor_last = 0.0
        self.monitor_a8_unsupported = False
        self.monitor_rates = RateTracker()
        self.video_progress = VideoProgressTracker()
        self.monitor_failures = 0
        self.monitor_context = None
        self.auto_policy = AutoCorrectionPolicy()
        self.auto_enabled = False
        self.auto_detail = '默认关闭；需当前 CNN 上下文、模型/K 就绪和新鲜预测。'
        self.auto_last_applied = None
        self.last_auto_proposal = None
        self.last_prediction_identity = None
        self.last_prediction_advanced = 0.0
        self.last_gamma_frame = None
        self.last_gamma_frame_changed = 0.0
        self.worker = ControlWorker(preview, run_dir, args)
        self.worker.auto_policy = self.auto_policy
        self.worker.message.connect(self.on_message, Qt.ConnectionType.QueuedConnection)
        self.worker.finished.connect(self.on_stopped, Qt.ConnectionType.QueuedConnection)
        window.connectRequested.connect(self.connect_board)
        window.angleRequested.connect(self.apply_angle)
        window.correctRequested.connect(self.correct)
        window.cropRequested.connect(self.set_crop)
        window.framePainted.connect(self.on_frame_painted)
        window.previewRetryRequested.connect(self.retry_preview)
        window.closingRequested.connect(self.close_requested)
        window.monitorRefreshRequested.connect(self.manual_refresh_monitor)
        window.monitorContextRequested.connect(self.check_monitor_context)
        window.modelLoadRequested.connect(self.model_load)
        window.modelKRequested.connect(self.model_k)
        window.modelEnableRequested.connect(self.model_enable)
        window.modelPrepareRequested.connect(self.model_prepare)
        window.automaticCorrectionRequested.connect(self.set_auto_correction)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.timer.start(16)  # poll latest, present only new source packets; no frame-event queue
        self.worker.start()
        self.dispatch('preview')
        self.window.set_monitor_state(self.monitor_state)
        self.window.set_monitor_context(False, '需要 algorithm1080_nn_r1 的匹配验收和 SRAM 编程会话；旧 R8 会话不适用。')
        self.window.set_auto_correction_state(False, self.auto_detail)

    def refresh_controls(self):
        self.window.set_control_state(self.control['connected'], self.busy and self.active_kind != 'monitor',
                                      self.control['angle'], self.state_text,
                                      control_ready=self.control.get('control_ready', self.control['connected']))
        crop = self.control.get('crop', {})
        detail = '去边状态未知；请先连接或重新应用角度'
        if crop.get('known'):
            if crop.get('enabled'):
                zoom = float(crop['zoom'])
                detail = f'已确认 {zoom:.3f}× · 视野线性保留约 {100/zoom:.1f}%'
            else:
                detail = '未去边：保留完整视野，边缘效果取决于当前算法'
        pending_crop = self.pending_crop
        if pending_crop is None and self.busy and self.active_kind == 'crop':
            pending_crop = bool(self.active_value)
        if pending_crop is not None:
            desired = '开启' if pending_crop else '关闭'
            detail += f' · 待确认{desired}去边'
        self.window.set_crop_state(crop.get('enabled') if crop.get('known') else None,
                                   detail, pending_crop)
        desired = self.pending_angle if self.pending_angle is not None else (self.active_value if self.busy and self.active_kind == 'angle' else None)
        self.window.set_pending_angle(float(desired) if desired is not None else None)
        self._refresh_auto_availability()

    def _refresh_auto_availability(self):
        cnn = self.monitor_state.get('cnn') or {}
        if not self.control.get('connected'):
            ready, reason = False, '请先连接控制板'
        elif not self.control.get('control_ready'):
            ready, reason = False, '板卡控制协议未就绪'
        elif self.monitor_state.get('freshness') != 'fresh':
            ready, reason = False, '正在读取 CNN 状态'
        elif (int(cnn.get('capability_word', 0)) >> 16) < 15 or not (int(cnn.get('capability_word', 0)) & 1):
            ready, reason = False, 'CNN 接口需使用修复版位流'
        elif not cnn.get('loaded'):
            ready, reason = False, '先点击“准备并启用 CNN”'
        elif str(cnn.get('crc32', '')).lower().removeprefix('0x') != '4ce634dd':
            ready, reason = False, '模型校验未通过'
        elif not cnn.get('enabled') or not cnn.get('k_ready'):
            ready, reason = False, '请点击“准备并启用 CNN”'
        elif not cnn.get('prediction_count'):
            ready, reason = False, '等待首次预测'
        elif not cnn.get('prediction_valid'):
            ready, reason = False, '当前预测无效，暂不应用'
        else:
            ready, reason = True, '已就绪，可开启自动校正'
        self.window.set_auto_availability(ready, reason)
        fw_ready = ((int(cnn.get('capability_word', 0)) >> 16) >= 15
                    and bool(int(cnn.get('capability_word', 0)) & 1))
        self.window.set_prepare_availability(fw_ready, '需要修复版 CNN 接口' if not fw_ready else '')

    def dispatch(self, kind, value=None):
        if self.pending_close or self.shutting_down:
            return
        if self.busy:
            if self.active_kind == 'monitor' and kind != 'monitor':
                self.pending_action = (kind, value)
            return
        self.active_kind, self.active_value = kind, value
        self.busy = True
        self.busy_since = time.monotonic()
        if kind != 'monitor':
            self.state_text = '正在连接……' if kind == 'connect' else '正在处理，请等待板卡确认……'
            self.refresh_controls()
        if kind != 'monitor':
            self.window.show_notice('正在处理', '预览会持续更新，生效角度以板卡回读为准。')
        self.active_kind = kind
        self.worker.submit(kind, value)
        if kind == 'nn_action':
            self.window.set_monitor_busy(True)
            self.window.set_monitor_state({**self.monitor_state, 'freshness': 'paused',
                                           'detail': '正在准备 CNN，监测暂时暂停'})

    def refresh_monitor(self):
        if self.pending_close or self.shutting_down:
            return
        if not self.control.get('connected'):
            self.monitor_state = {'freshness': 'unknown', 'detail': '请先连接控制板', 'cnn': {}, 'gamma': {}}
            self.window.set_monitor_state(self.monitor_state)
            return
        if self.busy or self.pending_angle is not None or self.pending_crop is not None or self.pending_action is not None:
            return
        self.monitor_last = time.monotonic()
        self.dispatch('monitor')

    def manual_refresh_monitor(self):
        self.monitor_a8_unsupported = False
        self.worker.monitor_a8_unsupported = False
        self.refresh_monitor()

    def _monitor_result(self, data):
        now = time.monotonic()
        cnn, gamma = data.get('cnn'), data.get('gamma')
        failed = not cnn and not gamma and not data.get('video')
        self.monitor_failures = self.monitor_failures + 1 if failed else 0
        if self.monitor_failures >= 3:
            self.worker.controller = self.worker.console = None
            self.control = {'connected': False, 'control_ready': False, 'angle': None, 'ack': None}
            self._manual_takeover('板卡连续未响应')
            self.state_text = '板卡连续未响应，请重新连接'
        video = self.video_progress.observe(data.get('video'), now)
        self.window.set_video_health(video['state'], video['detail'])
        if video['state'] == 'stalled':
            if self.worker.controller:
                self.worker.controller.video_stalled = True
            self.control['control_ready'] = False
            self._manual_takeover('板卡处理帧已停止')
            self.state_text = video['detail']
        elif video['state'] == 'running' and self.worker.controller:
            self.worker.controller.video_stalled = False
            self.control = self.worker.state()
        freshness, details = 'fresh', []
        if cnn:
            identity = (cnn.get('crc32'), cnn.get('loaded'), cnn.get('k_ready'), cnn.get('enabled'))
            cnn['predictions_per_sec'] = self.monitor_rates.rate(cnn.get('prediction_count') or 0, now,
                identity=identity, ready=bool(cnn.get('loaded') and cnn.get('k_ready')))
            if cnn.get('prediction_count') == 0:
                cnn['predicted_angle_deg'] = None
                cnn['corners'] = []
            if cnn.get('protocol_error') or cnn.get('fault_latched') or cnn.get('saturated') or cnn.get('nn_error'):
                details.append('CNN 存在设备错误/饱和/故障标志')
                freshness = 'error'
            prediction_identity = (cnn.get('crc32'), cnn.get('prediction_count'),
                                   (cnn.get('decoded') or {}).get('telemetry', {}).get('predictionFrame'))
            if prediction_identity != self.last_prediction_identity:
                self.last_prediction_identity = prediction_identity
                self.last_prediction_advanced = now
            if now - self.last_prediction_advanced > 8.0:
                cnn['prediction_valid'] = False
                cnn['predicted_angle_deg'] = None
                cnn['corners'] = []
                if self.auto_enabled:
                    self.auto_policy.refused()
        if data.get('a7_error'):
            freshness = 'error'; details.append('A7：' + data['a7_error'])
        if data.get('a5_error'):
            details.append('视频处理状态暂未读到')
        if gamma:
            gamma['updated_monotonic'] = now
            self.monitor_a8_unsupported = False
            if not gamma.get('valid'):
                gamma = {'supported': True, 'valid': False, 'status': '尚无当前配置的已提交统计',
                         'enabled': None, 'applied_bank': None, 'next_bank': None,
                         'mean_luminance': None, 'valid_count': None,
                         'frame_id': gamma.get('frame_id'), 'config_version': gamma.get('config_version'),
                         'updated_monotonic': now}
            else:
                frame_id = gamma.get('frame_id')
                if frame_id != self.last_gamma_frame:
                    self.last_gamma_frame = frame_id
                    self.last_gamma_frame_changed = now
                ack = self.control.get('ack') or {}
                ack_version = ack.get('activeVersion')
                stale_reason = None
                if now - self.last_gamma_frame_changed > 3.0:
                    stale_reason = 'Gamma 来源帧超过 3 秒未变化'
                elif (gamma.get('config_version') != 0xffff and ack_version is not None
                      and (int(gamma.get('config_version', -1)) & 0xffff) != (int(ack_version) & 0xffff)):
                    stale_reason = 'Gamma 统计对应旧配置版本'
                if stale_reason:
                    gamma = {**gamma, 'status': stale_reason, 'enabled': None,
                             'applied_bank': None, 'next_bank': None,
                             'mean_luminance': None, 'valid': False, 'stale': True}
        elif data.get('a8_error'):
            details.append('A8：' + data['a8_error'])
            if 'magic/version mismatch' in data['a8_error']:
                self.monitor_a8_unsupported = True
                self.worker.monitor_a8_unsupported = True
            else:
                gamma = {'status': '统计读取暂时失败，将自动重试', 'error': data['a8_error']}
        if self.monitor_a8_unsupported:
            gamma = {'supported': False, 'status': '未取得有效 A8，请刷新重试'}
        if not cnn and not gamma:
            freshness = 'error'
        if self.monitor_a8_unsupported and not gamma:
            details.append('此固件不提供 A8；停止自动探测，可手动刷新重试。')
        self.monitor_state = {'freshness': freshness, 'detail': ' · '.join(details) or '来自串口设备快照',
                              'cnn': cnn or {}, 'gamma': gamma or {}, 'context': {'qualified': False,
                              'detail': '需要匹配的 algorithm1080_nn_r1 验收记录和编程会话；只读监测不需要该上下文。'}}
        self.window.set_monitor_state(self.monitor_state)
        if self.auto_enabled:
            qualified = bool(self.control.get('control_ready') and cnn and cnn.get('loaded') and cnn.get('enabled')
                             and cnn.get('k_ready') and cnn.get('prediction_valid')
                             and str(cnn.get('crc32', '')).lower().removeprefix('0x') == '4ce634dd'
                             and now - self.last_prediction_advanced <= 8.0
                             and self.monitor_state['freshness'] == 'fresh')
            if not qualified:
                self.auto_detail = '自动校正暂停：当前上下文、模型/K 就绪或新鲜有效预测不满足要求。'
                self.window.set_auto_correction_state(False, self.auto_detail)
                self.auto_enabled = False
                self.worker.auto_enabled = False
                self.auto_policy.disable()
            elif not self.pending_close and self.pending_angle is None and self.pending_crop is None:
                wrapped = {'decoded': cnn['decoded'], 'raw_packet_hex': cnn['raw_a7_hex'], 'observed_at': now}
                proposal = self.auto_policy.observe(wrapped, now=now)
                if proposal is not None:
                    self.last_auto_proposal = proposal
                    self.auto_detail = (f"稳定预测满足条件：目标角度 {proposal['target_angle_deg']:+.2f}°；"
                                        '等待自动校正 ACK（至少间隔 5 秒）。')
                    self.window.set_auto_correction_state(self.auto_enabled, self.auto_detail if self.auto_enabled else
                        '该自动配置已由 ACK 确认；自动模式仍关闭，手动目标将在随后执行。')
                    self.dispatch('auto_apply', proposal)
                else:
                    self.auto_detail = '自动校正已开启；等待 3 次连续稳定的新鲜 CNN 预测。'
                    self.window.set_auto_correction_state(True, self.auto_detail)

    def check_monitor_context(self):
        self._manual_takeover('控制资格发生变更')
        if self.busy or self.pending_close or self.shutting_down:
            self.window.show_notice('当前操作完成后再检查资格', 'R14 控制上下文不能在串口事务进行时切换。', 'warning')
            return
        acceptance, _ = QFileDialog.getOpenFileName(self.window, '选择 algorithm1080_nn_r1 验收记录', str(HERE.parents[1]), 'JSON (*.json)')
        if not acceptance:
            return
        session, _ = QFileDialog.getOpenFileName(self.window, '选择匹配的 SRAM 编程会话', str(Path(acceptance).parent), 'JSON (*.json)')
        if not session:
            return
        try:
            from auto_correction import _validate_r14_program_binding
            qualified, program_session = _validate_r14_program_binding(Path(acceptance), Path(session))
            prior_context = self.monitor_context
            self.monitor_context = (Path(acceptance), Path(session))
            self.worker.monitor_context = self.monitor_context
            if prior_context != self.monitor_context and self.control.get('connected'):
                self.worker.controller = None
                self.worker.console = None
                self.control = {'connected': False, 'angle': None, 'ack': None,
                                'crop': {'known': False, 'enabled': None, 'zoom': None}}
                self.refresh_controls()
            self.window.set_monitor_context(True, f"已绑定候选 {qualified['candidate']} 与会话 {program_session.get('sessionId')}。模型写入仅由明确按钮触发。")
        except Exception as exc:
            self.monitor_context = None
            self.window.set_monitor_context(False, f'资格检查失败：{exc}')

    def model_load(self):
        self._submit_model_action({'kind': 'load'})

    def model_prepare(self):
        self._manual_takeover('准备 CNN')
        if not self.control.get('live_control'):
            self.window.show_notice('请先连接控制板', '连接后再准备 CNN。', 'warning')
            return
        self.dispatch('nn_action', {'kind': 'prepare'})

    def model_k(self):
        values = []
        for label in ('fx', 'fy', 'cx', 'cy'):
            value, ok = QInputDialog.getInt(self.window, '配置 K（S(16,11) 原始寄存器）', f'{label} 原始有符号16位值：', 1 if label in ('fx','fy') else 0, -32768, 32767)
            if not ok: return
            values.append(value)
        if values[0] <= 0 or values[1] <= 0:
            self.window.set_monitor_context(bool(self.monitor_context), 'K 的 fx/fy 必须为正值；采用用户手工提供的原始 S(16,11) 寄存器值，不做自动缩放。')
            return
        self._submit_model_action({'kind': 'k', 'values': values})

    def model_enable(self, enabled):
        self._submit_model_action({'kind': 'enable', 'value': bool(enabled)})

    def _submit_model_action(self, action):
        self._manual_takeover('手动模型控制')
        if not self.monitor_context:
            self.check_monitor_context()
            if not self.monitor_context:
                return
        if not self.control.get('connected'):
            self.window.show_notice('请先连接', '模型操作需要当前连接的控制板。', 'warning')
            return
        self.dispatch('nn_action', action)

    def set_auto_correction(self, enabled):
        if not enabled:
            self._manual_takeover('用户已关闭 CNN 自动校正')
            return
        cnn = self.monitor_state.get('cnn') or {}
        if (not self.control.get('control_ready') or self.monitor_state.get('freshness') != 'fresh'
                or (int(cnn.get('capability_word', 0)) >> 16) < 15
                or not (int(cnn.get('capability_word', 0)) & 1)
                or not cnn.get('loaded') or not cnn.get('enabled') or not cnn.get('k_ready')
                or not cnn.get('prediction_valid')
                or str(cnn.get('crc32', '')).lower().removeprefix('0x') != '4ce634dd'):
            self.auto_enabled = False
            self.worker.auto_enabled = False
            self.auto_policy.disable()
            self.auto_detail = '暂不能启用：需匹配的当前候选上下文、已加载并启用的模型、有效 K 和新鲜有效预测。'
            self.window.set_auto_correction_state(False, self.auto_detail)
            return
        if not self.control.get('connected'):
            self.auto_detail = '暂不能启用：请先连接控制板。'
            self.window.set_auto_correction_state(False, self.auto_detail)
            return
        self.auto_policy.enable()
        self.worker.auto_enabled = True
        self.auto_enabled = True
        self.auto_detail = '自动校正已启用；等待 3 次连续稳定的新鲜 CNN 预测。手动角度、去边或模型操作会立即关闭。'
        self.window.set_auto_correction_state(True, self.auto_detail)

    def _manual_takeover(self, reason):
        self.worker.auto_enabled = False
        if self.auto_enabled:
            self.auto_enabled = False
            self.auto_policy.disable()
            self.auto_detail = f'自动校正已关闭：{reason}。'
            self.window.set_auto_correction_state(False, self.auto_detail)
        self.last_auto_proposal = None

    @Slot(str)
    def connect_board(self, port):
        self._manual_takeover('重新连接控制板')
        self.dispatch('connect', port)

    @Slot(str)
    def apply_angle(self, angle):
        if self.pending_close or self.shutting_down:
            return
        try:
            command, value = parse_command(str(angle))
            if command != 'angle' or not math.isfinite(float(value)):
                raise ValueError('请输入有限的角度数值。')
        except (ValueError, TypeError, OverflowError) as exc:
            self.window.show_notice('角度无效', str(exc), 'warning')
            return
        if not self.control['connected']:
            self.window.show_notice('请先连接', '连接控制板后可调整角度。', 'warning')
            return
        normalized = format(float(value), '.12g')
        self._manual_takeover('手动角度输入')
        if self.busy:
            self.pending_angle = normalized
            self.window.set_pending_angle(float(value))
            self.window.show_notice('已更新目标角度', '当前操作完成后直接应用最新角度，中间目标不会排队。')
            return
        self.pending_angle = None
        self.dispatch('angle', normalized)

    @Slot()
    def correct(self):
        self._manual_takeover('手动自动摆正操作')
        self.dispatch('correct')

    @Slot(bool)
    def set_crop(self, enabled):
        self._manual_takeover('手动去边设置')
        if self.pending_close or self.shutting_down:
            return
        if not self.control.get('connected'):
            self.window.show_notice('请先连接', '连接控制板后可更改旋转去边。', 'warning')
            return
        self.pending_crop = bool(enabled)
        self.refresh_controls()
        if self.busy:
            self.window.show_notice('已更新去边目标', '当前操作完成后应用最新的去边设置。')
            return
        self._dispatch_pending_crop()

    def _dispatch_pending_crop(self):
        if (self.pending_crop is None or self.busy or self.pending_close or self.shutting_down
                or not self.control.get('connected')):
            return
        crop = self.control.get('crop') or {}
        if not crop.get('known'):
            self.refresh_controls()
            self.window.show_notice('去边状态未知', '请先应用角度并等待板卡确认，再更改去边。', 'warning')
            return
        desired, self.pending_crop = self.pending_crop, None
        if bool(crop.get('enabled')) == desired:
            self.refresh_controls()
            self.window.show_notice('去边状态已确认', '板卡当前状态已符合最新去边目标，无需重复下发。', 'success')
            return
        self.dispatch('crop', desired)

    @Slot()
    def retry_preview(self):
        self.dispatch('preview')

    @Slot(object)
    def on_message(self, message):
        kind = message['type']
        if kind == 'log':
            self.window.append_log(message['text'])
            self.state_text = message['text']
            self.refresh_controls()
        elif kind == 'done':
            self.busy = False
            self.control = message['state']
            if message.get('kind') == 'connect' and self.control.get('connected'):
                self.monitor_a8_unsupported = False
                self.worker.monitor_a8_unsupported = False
                self.video_progress = VideoProgressTracker()
                self.monitor_failures = 0
            title, detail, level = message['notice']
            if message.get('kind') == 'monitor':
                self._monitor_result(message.get('result') or {})
            elif message.get('kind') == 'auto_apply':
                result = message.get('result') or {}
                if result.get('status') == 'APPLIED':
                    proposal = self.active_value if isinstance(self.active_value, dict) else self.last_auto_proposal
                    was_auto_enabled = self.auto_enabled
                    if was_auto_enabled:
                        self.auto_policy.applied(proposal, time.monotonic())
                    applied = result.get('applied') or {}
                    self.auto_last_applied = applied
                    angle = proposal.get('target_angle_deg') if proposal else None
                    board_angle = self.worker.controller.current_angle if self.worker.controller else None
                    board_angle_text = f'{board_angle:+.2f}°' if board_angle is not None else '未知'
                    self.auto_detail = (f"CNN 自动校正 ACK 已确认：CNN 补偿角 {angle:+.2f}°；"
                        f"板卡 ACK 实际角 {board_angle_text} · 序列 {applied.get('appliedSequence')} · 配置版本 {applied.get('activeVersion')}。"
                        '预测与 ACK 生效状态分开记录。')
                    warning = (result.get('provenance') or {}).get('persistenceWarning')
                    if warning:
                        self.auto_detail += f' · 记录警告：{warning}'
                    self.window.set_auto_correction_state(was_auto_enabled, self.auto_detail if was_auto_enabled else
                        '本次自动更新已由 ACK 确认；自动模式保持关闭，排队的手动目标随后执行。')
                    self.control['angle'] = self.worker.controller.current_angle if self.worker.controller else None
                    self.control['ack'] = self.worker.controller.last_applied if self.worker.controller else None
                    self.monitor_state['last_auto_applied'] = {'target_angle_deg': angle, 'applied': applied,
                        'provenance': result.get('provenance')}
                    self.window.set_monitor_state(self.monitor_state)
                elif result.get('ownership_lost'):
                    self.auto_policy.failed()
                    self.auto_enabled = False
                    self.worker.auto_enabled = False
                    self.control['angle'] = None
                    self.control['ack'] = None
                    self.control['crop'] = {'known': False, 'enabled': None, 'zoom': None}
                    self.monitor_state.update(freshness='unknown', detail='板卡配置所有权检查失败；角度、ACK 和去边状态未知。')
                    self.window.set_monitor_state(self.monitor_state)
                    self.window.set_auto_correction_state(False, '自动校正已暂停：板卡配置所有权发生变化，需重新连接并确认。')
                elif result.get('uncertain') is False:
                    if message['state'].get('angle') is None or message['state'].get('ack') is None:
                        self.auto_policy.failed()
                        self.auto_enabled = False
                        self.worker.auto_enabled = False
                        self.control['angle'] = None
                        self.control['ack'] = None
                        self.auto_detail = '外部所有权变化使原 ACK 无法确认；自动更新已暂停。'
                        self.window.set_auto_correction_state(False, self.auto_detail)
                    else:
                        self.auto_detail = f"自动预测未通过发送前检查，保留上次 ACK 配置：{result.get('error', '检查失败')}"
                        self.window.set_auto_correction_state(self.auto_enabled, self.auto_detail)
                else:
                    self.auto_policy.failed()
                    self.auto_enabled = False
                    self.worker.auto_enabled = False
                    self.control['angle'] = None
                    self.control['ack'] = None
                    self.monitor_state.update(freshness='unknown', detail='自动校正 ACK 不确定；角度和配置状态未知。')
                    self.window.set_monitor_state(self.monitor_state)
                    self.window.set_auto_correction_state(False, '自动校正已暂停：ACK 不确定，需手动重新建立确认状态。')
            else:
                if message.get('kind') == 'nn_action':
                    self.window.set_monitor_busy(False)
                self.state_text = title
                self.window.show_notice(title, detail, level)
                self.window.append_log(f'[{datetime.now().strftime("%H:%M:%S")}] {title}：{detail}')
            self.refresh_controls()
            if self.pending_close:
                self.pending_angle = None
                self.pending_crop = None
                self.pending_action = None
                self.window.set_pending_angle(None)
                self.begin_shutdown()
            elif self.pending_angle is not None and self.control['connected']:
                newest, self.pending_angle = self.pending_angle, None
                # Same GUI-event turn: no stale deferred callback can replace a newer target.
                self.dispatch('angle', newest)
            elif self.pending_crop is not None:
                self._dispatch_pending_crop()
            elif self.pending_action is not None:
                next_action, self.pending_action = self.pending_action, None
                self.dispatch(*next_action)
            elif message.get('kind') in ('nn_action', 'connect'):
                self.refresh_monitor()
        elif kind == 'shutdown':
            self.control = message['state']
        elif kind == 'shutdown_failed':
            self.busy = self.pending_close = self.shutting_down = False
            self.state_text = '视频资源尚未释放，请再次关闭以重试'
            self.window.show_notice('暂未退出', message['error'], 'error')
            self.refresh_controls()

    @Slot(object)
    def on_frame_painted(self, identity):
        if not isinstance(identity, tuple) or len(identity) != 2:
            return
        generation, sequence = identity
        if identity == self.last_qt_painted_identity:
            return
        if (self.last_qt_painted_identity is not None
                and self.last_qt_painted_identity[0] == generation):
            self.qt_paint_skipped_frames += max(
                0, sequence - self.last_qt_painted_identity[1] - 1)
        self.last_qt_painted_identity = identity
        self.paint_times.append(time.monotonic())
        self.qt_painted_frames += 1

    @Slot()
    def tick(self):
        preview = self.preview.status()
        latest = self.preview.latest_with_identity()
        if latest is not None:
            generation, sequence, stamp, frame = latest
            identity = (generation, sequence)
            if identity != self.latest_sequence:
                self.window.show_frame(frame, identity)
                if self.latest_sequence is not None and self.latest_sequence[0] == identity[0]:
                    self.host_submission_skipped_frames += max(0, sequence-self.latest_sequence[1]-1)
                self.latest_sequence = identity
                self.frame_submissions += 1
            now = time.monotonic()
            while self.paint_times and now-self.paint_times[0] > 2:
                self.paint_times.popleft()
            painted_fps = ((len(self.paint_times)-1)/(self.paint_times[-1]-self.paint_times[0])
                           if len(self.paint_times) >= 2 else 0.0)
            self.window.set_preview_status('实时预览',
                f'1920 × 1080 · 解码 {preview["fps"]:.1f} / Qt绘制 {painted_fps:.1f} FPS · 目标30')
            if not self.snapshot_saved and self.window.isVisible():
                self.window.grab().save(str(self.run_dir/'interface_screenshot.png'))
                self.snapshot_saved = True
        else:
            starting = preview['state'] in ('starting', 'stopped')
            self.window.set_preview_status('等待视频' if starting else '预览已中断',
                '请关闭其他相机预览后点击“重试预览”；若无设备，请检查HDMI采集卡USB连接。')
        now = time.monotonic()
        if self.busy and self.active_kind != 'monitor' and not self.shutting_down:
            suffix = f' · 已用时 {int(now-self.busy_since)} 秒'
            self.window.set_control_state(self.control['connected'], True, self.control['angle'], self.state_text+suffix,
                control_ready=self.control.get('control_ready', self.control['connected']))
        if self.worker.args.offline_demo:
            self.window.set_preview_status('离线演示 · 非板卡画面', '仅检查界面，不占用采集卡或串口。')
        if now-self.last_status_write >= 1.0:
            self.last_status_write = now
            try:
                write_json(self.run_dir/'app_status.json', {
                    'updatedUtc': utc(), 'pid': os.getpid(), 'windowVisible': self.window.isVisible(),
                    'busy': self.busy, 'pendingClose': self.pending_close, 'closed': False,
                    'pendingTargetAngle': self.pending_angle, 'activeOperation': self.active_kind,
                    'control': self.control, 'preview': preview,
                    'latestWidgetSubmissionIdentity': self.latest_sequence, 'hardwareGuiAcceptance': False,
                    'frameSubmissionsToWidget': self.frame_submissions,
                    'hostSubmissionSkippedFrames': self.host_submission_skipped_frames,
                    'qtPaintedFrames': self.qt_painted_frames,
                    'qtPaintSkippedFrames': self.qt_paint_skipped_frames,
                    'lastQtPaintedIdentity': self.last_qt_painted_identity,
                    'viewMode': self.window.view_mode(),
                    'qtPaintFpsRecent': ((len(self.paint_times)-1)/(self.paint_times[-1]-self.paint_times[0])
                                         if latest is not None and len(self.paint_times) >= 2 else 0),
                    'qtPaintFpsMeasuresWidgetPaintEventsNotMonitorVsync': True,
                })
            except OSError:
                pass
        if (self.control.get('connected')
                and not self.busy and not self.pending_close and not self.shutting_down
                and self.pending_angle is None and self.pending_crop is None
                and now - self.monitor_last >= 1.0):
            self.refresh_monitor()
        if (self.monitor_state.get('freshness') == 'fresh' and now - self.monitor_last > 3.0
                and not (self.busy and self.active_kind != 'monitor')):
            self.monitor_state['freshness'] = 'stale'
            self.monitor_state['detail'] = '设备快照超过 3 秒未更新'
            self.window.set_monitor_state(self.monitor_state)

    @Slot()
    def close_requested(self):
        if self.pending_close or self.shutting_down:
            self.window.show_notice('正在安全退出', '等待当前任务结束并释放本程序的视频采集。', 'warning')
            return
        self._manual_takeover('关闭窗口')
        if self.busy and self.active_kind != 'monitor':
            choice = QMessageBox.question(self.window, '等待任务结束后退出？',
                '正在校验或下发参数。为避免中断实际操作，确认后会在任务结束时自动关闭。',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No)
            if choice != QMessageBox.StandardButton.Yes:
                return
            self.pending_close = True
            self.pending_crop = None
            self.window.show_notice('等待完成后退出', '当前操作会继续，预览仍可查看。', 'warning')
        elif self.busy:
            self.pending_close = True
            self.pending_action = None
            self.pending_angle = self.pending_crop = None
        else:
            self.pending_crop = None
            self.begin_shutdown()

    def begin_shutdown(self):
        if self.shutting_down:
            return
        self.pending_angle = None
        self._manual_takeover('安全退出')
        self.window.set_pending_angle(None)
        self.shutting_down = True
        self.pending_close = True
        self.busy = True
        self.state_text = '正在释放视频采集……'
        self.refresh_controls()
        self.worker.submit('shutdown')

    @Slot()
    def on_stopped(self):
        self.timer.stop()
        record = {'closedUtc': utc(), 'closed': True, 'pid': os.getpid(), 'state': self.control,
                  'preview': self.preview.status(), 'retainsLastAppliedAngle': True}
        try:
            write_json(self.run_dir/'closed.json', record)
            write_json(self.run_dir/'app_status.json', record)
        except OSError:
            pass
        self.window.allow_close()


def single_instance():
    if os.name != 'nt':
        return None
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.CreateMutexW.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p]
    kernel.CreateMutexW.restype = ctypes.c_void_p
    ctypes.set_last_error(0)
    handle = kernel.CreateMutexW(None, False, 'Local\\FPGA_Rotation_Dashboard_v1')
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    if ctypes.get_last_error() == 183:
        kernel.CloseHandle.argtypes = [ctypes.c_void_p]
        kernel.CloseHandle(handle)
        return False
    return handle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline-demo', action='store_true', help='Generated UI preview only; no camera/UART access')
    parser.add_argument('--smoke-exit-ms', type=int, default=0)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--device', default='USB Video')
    parser.add_argument('--work-dir', type=Path, default=HERE/'runtime')
    args = parser.parse_args()
    if not __debug__:
        raise RuntimeError('Do not use Python -O with qualified board tools')
    for p in [args.profile, args.work_dir]:
        if not str(p.resolve()).isascii():
            raise ValueError('工程和记录路径必须使用英文路径。')
    app = QApplication(sys.argv[:1])
    app.setApplicationName('FPGA Rotation Dashboard V5')
    from PySide6.QtGui import QIcon
    if (HERE/'icon.svg').is_file():
        app.setWindowIcon(QIcon(str(HERE/'icon.svg')))
    mutex = single_instance()
    if mutex is False:
        QMessageBox.information(None, '界面已经打开', '请先关闭已经打开的控制台，再启动此版本，避免两个窗口同时占用串口。')
        return 0
    run_dir = args.work_dir/('gui_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    run_dir.mkdir(parents=True, exist_ok=False)
    write_json(run_dir/'startup.json', {'startedUtc': utc(), 'argv': sys.argv, 'pid': os.getpid(),
        'automaticallyProgramsBoard': False, 'configRequiresExplicitConnect': True,
        'legacyConsole': str(LEGACY), 'realPreviewOnly': True})
    window = DashboardWindow()
    if args.offline_demo:
        from demo_preview import DemoPreview
        preview = DemoPreview()
    else:
        preview = PreviewStream(args.device, run_dir/'preview')
    coordinator = Coordinator(window, preview, run_dir, args)
    window.show()
    if args.smoke_exit_ms:
        QTimer.singleShot(max(500, args.smoke_exit_ms), coordinator.close_requested)
    try:
        return app.exec()
    finally:
        # The window's normal close waits for worker.finished. Do not terminate
        # qualified send subprocesses; normal lifecycle is handled above.
        if mutex:
            kernel = ctypes.WinDLL('kernel32', use_last_error=True)
            kernel.CloseHandle.argtypes = [ctypes.c_void_p]
            kernel.CloseHandle(mutex)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception:
        log = HERE/'startup_error.log'
        log.write_text(traceback.format_exc(), encoding='utf-8')
        if os.name == 'nt':
            ctypes.windll.user32.MessageBoxW(None, f'界面启动失败。错误记录：\n{log}', 'FPGA旋转面板', 0x10)
        raise
