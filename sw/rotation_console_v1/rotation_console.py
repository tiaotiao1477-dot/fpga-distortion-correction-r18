"""Interactive PC UART angle commands for the existing qualified FPGA pipeline."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import uuid

sys.dont_write_bytecode = True

from capture_frames import capture_batch
from qualified_control import (ExternalStateChanged, QualifiedController,
                               discover_session, program_new_session)
from roll_estimator import estimate_roll

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_BOARD = WORKSPACE / "wanchengceshi/algorithm_v04_1080p_upgrade/algorithm1080_rejoin_r8"
DEFAULT_PROFILE = WORKSPACE / "data/calibration/ov5640_user_photos_20261022/board_validation_20261023/measured_native_profile/camera_profile_1080_runtime.json"


def utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def normalize(angle: float) -> float:
    if not math.isfinite(angle):
        raise ValueError("角度必须是有限数字，例如20、-15、13.37。")
    result = math.remainder(angle, 360.0)
    return 0.0 if result == 0 else result


def parse_command(text: str) -> tuple[str, float | None]:
    value = text.strip()
    lower = value.lower()
    if lower in {"correct", "status", "help", "quit", "exit"}:
        return ("quit" if lower == "exit" else lower), None
    if not value:
        return "empty", None
    number = re.sub(r"\s*(deg|度)\s*$", "", value, flags=re.IGNORECASE)
    try:
        return "angle", normalize(float(number))
    except (ValueError, OverflowError):
        raise ValueError("请输入数字角度、correct、status、help或quit。") from None


def brief_state(controller: QualifiedController) -> dict:
    return {"known_angle_degrees": controller.current_angle,
            "acknowledged_configuration": controller.last_applied}


class RotationConsole:
    def __init__(self, controller: QualifiedController, run_dir: Path, device: str):
        self.controller = controller
        self.run_dir = run_dir
        self.device = device

    def command_dir(self, label: str) -> Path:
        path = self.run_dir / (label + "_" + uuid.uuid4().hex)
        path.mkdir(exist_ok=False)
        return path

    def numeric(self, angle: float, label: str = "angle") -> None:
        folder = self.command_dir(label)
        record = {"command": "angle", "requested_absolute_degrees": angle,
                  "startedUtc": utc(), "before": brief_state(self.controller)}
        try:
            print(f"正在校验并设置绝对角度 {angle:g}°，请等待实际生效确认……", flush=True)
            applied = self.controller.apply_angle(angle)
            record.update(status="APPLIED", result=applied, after=brief_state(self.controller))
            print(f"已生效：{angle:g}°，真实镜头校正保持开启。", flush=True)
        except BaseException as exc:
            record.update(status="FAILED_OR_UNKNOWN", error=repr(exc), after=brief_state(self.controller))
            raise
        finally:
            record["endedUtc"] = utc()
            write_json(folder / "result.json", record)

    def _capture_estimate(self, folder: Path, label: str, record: dict):
        self.controller.require_unchanged()
        capture = capture_batch(folder / label, device=self.device)
        # Another host or a reset during capture invalidates the angle used to
        # interpret the captured output. Never compensate against a stale angle.
        state = self.controller.require_unchanged()
        estimate = estimate_roll(capture.frames)
        write_json(folder / (label + "_estimate.json"), estimate.to_dict())
        record.setdefault("measurements", []).append({
            "capture_record": str(capture.record_path),
            "capture_record_sha256": hashlib.sha256(capture.record_path.read_bytes()).hexdigest(),
            "estimate": estimate.to_dict(),
            "observed_configuration": state["algorithmConfig"],
        })
        return estimate

    def correct(self) -> None:
        folder = self.command_dir("correct")
        record = {"command": "correct", "startedUtc": utc(),
                  "before": brief_state(self.controller), "writes": [],
                  "one_shot": True, "gravity_measured": False}
        prior_angle: float | None = None
        write_attempted = False
        try:
            self.controller.require_unchanged()
            prior_angle = self.controller.current_angle
            if prior_angle is None:
                raise RuntimeError("当前角度未知，请先输入0建立基准。")
            print("正在采样：优先找棋盘格，再判断场景直线，请保持画面稳定……", flush=True)
            estimate = self._capture_estimate(folder, "before", record)
            if not estimate.accepted:
                record.update(status="REFUSED_NO_RELIABLE_REFERENCE", reason=estimate.reason)
                print("无法可靠判断水平，当前角度保持。请保持画面静止，或放入水平参照棋盘格。", flush=True)
                return
            reference = estimate.method
            residual = float(estimate.roll_deg)
            print(f"参考：{'棋盘格' if reference == 'chessboard' else '场景直线'}；偏转 {residual:+.2f}°。", flush=True)
            if abs(residual) <= 1.0:
                record.update(status="ALREADY_LEVEL", residual_degrees=residual)
                print("当前已在1°范围内，无需调整。", flush=True)
                return
            # Native positive output rotation is counterclockwise. Positive
            # detected slope is clockwise in image coordinates, hence ADD.
            for iteration in range(2):
                self.controller.require_unchanged()
                known = self.controller.current_angle
                if known is None:
                    raise RuntimeError("配置状态未知，不能继续自动修正。")
                target = normalize(known + residual)
                print(f"正在设置 {target:+.2f}°，然后重新采样确认……", flush=True)
                write_attempted = True
                applied = self.controller.apply_angle(target)
                record["writes"].append({"iteration": iteration, "target_degrees": target,
                                          "apply_result": applied})
                after = self._capture_estimate(folder, f"verify_{iteration}", record)
                if not after.accepted or after.method != reference:
                    record["verification_failure"] = "reference_lost_or_changed"
                    break
                remaining = float(after.roll_deg)
                if abs(remaining) <= 1.0:
                    record.update(status="CORRECTED_VISUALLY_CONFIRMED", residual_degrees=remaining,
                                  target_degrees=target, method=reference)
                    print(f"已摆正：实际角度 {target:+.2f}°，复测剩余偏转 {remaining:+.2f}°。", flush=True)
                    return
                # Refine at most once, only after a clear improvement. This
                # prevents wrong-sign, changing-scene or stale-frame oscillation.
                if iteration == 0 and abs(remaining) < min(5.0, abs(residual) * .8):
                    residual = remaining
                    continue
                record["verification_failure"] = "residual_not_small_or_not_improving"
                break
            record["status"] = "VISUAL_VERIFICATION_FAILED"
            self._rollback(prior_angle, record)
        except ExternalStateChanged as exc:
            # A competing writer owns the new state; do not overwrite it by
            # restoring our earlier angle. It might be an intentional command.
            record.update(status="ABORTED_EXTERNAL_STATE_CHANGE", error=repr(exc))
            print("检测到外部配置变化或复位，自动调整已停止。请先输入一个数字角度重新建立基准。", flush=True)
        except KeyboardInterrupt:
            self.controller.current_angle = None
            self.controller.last_applied = None
            record.update(status="INTERRUPTED_STATE_UNKNOWN")
            raise
        except Exception as exc:
            record.update(status="FAILED", error=repr(exc))
            if write_attempted and prior_angle is not None:
                self._rollback(prior_angle, record)
            else:
                print(f"未执行自动旋转：{exc}", flush=True)
        finally:
            record.update(after=brief_state(self.controller), endedUtc=utc())
            write_json(folder / "result.json", record)

    def _rollback(self, angle: float, record: dict) -> None:
        # A failed/uncertain ACK clears local state. Never overwrite a board
        # configuration we can no longer identify as our own acknowledged write.
        try:
            self.controller.require_unchanged()
        except Exception as exc:
            record["rollback"] = {"acknowledged": False, "attempted": False,
                                  "reason": "configuration_ownership_unknown_or_changed",
                                  "error": repr(exc)}
            print("无法确认配置仍属于本次操作，未发送恢复命令。请检查状态后输入0。", flush=True)
            return
        try:
            result = self.controller.apply_angle(angle)
            record["rollback"] = {"acknowledged": True, "attempted": True,
                                  "angle_degrees": angle, "result": result}
            print("自动摆正未能确认，已通过回读确认恢复原角度。", flush=True)
        except Exception as exc:
            record["rollback"] = {"acknowledged": False, "attempted": True, "error": repr(exc)}
            print("自动摆正未确认，恢复原角度也未成功。当前角度未知；请检查连接后输入0。", flush=True)

    def status(self) -> None:
        result = self.controller.snapshot()
        folder = self.command_dir("status")
        write_json(folder / "result.json", result)
        decoded = result["decoded"]
        cfg = decoded["algorithmConfig"]
        print(f"本控制台最后确认角度：{self.controller.current_angle}°；"
              f"板上序号：{cfg['appliedSequence']}，版本：{cfg['activeVersion']}。")
        print(f"RAW：{decoded['rawWidthMinPixels']}×{decoded['rawFrameLines']}；"
              f"累计源丢帧：{decoded['sourceFrameDropCount']}。累计值不是本次窗口增量。")
        if self.controller.last_applied and any(cfg[k] != self.controller.last_applied[k]
            for k in ("receivedSequence", "appliedSequence", "activeVersion")):
            self.controller.current_angle = None
            self.controller.last_applied = None
            print("板上配置已变化，本地角度失效；请先输入数字角度。")


def show_help() -> None:
    print("""
输入数字：设置绝对角度，正值逆时针，如20、-15、13.37。
0       ：真实镜头校正，取消额外旋转。
correct ：一次自动摆正，优先棋盘格，其次场景横竖线；不会连续跟踪。
status  ：读取当前诊断。
help    ：显示帮助。
quit    ：退出，保留最后已应用角度。

棋盘格应作为你认可的水平参照。没有传感器时，场景线条不能保证代表重力方向，
也不能可靠识别180°上下倒置。判断不可靠时程序会拒绝自动调整。
""", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board-root", type=Path, default=DEFAULT_BOARD)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--port", default="COM14")
    parser.add_argument("--device", default="USB Video")
    parser.add_argument("--work-dir", type=Path, default=Path(__file__).resolve().parent / "runtime")
    parser.add_argument("--program", action="store_true", help="Explicitly reload the existing qualified SRAM image before connecting")
    args = parser.parse_args()
    if not __debug__:
        raise RuntimeError("Do not use Python -O; existing qualified tools require assertions")
    for value in (args.board_root, args.profile, args.work_dir):
        if not str(value.resolve()).isascii():
            raise ValueError("工程和运行记录路径必须不含中文。")
    args.work_dir.mkdir(parents=True, exist_ok=True)
    run_dir = args.work_dir / ("console_" + datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8])
    run_dir.mkdir(exist_ok=False)
    print("连接前会核对完整合格文件。启动后先加载真实0°校正，建立已知角度。", flush=True)
    session = program_new_session(args.board_root) if args.program else (
        args.session or discover_session(args.board_root))
    write_json(run_dir / "startup.json", {"startedUtc": utc(), "session": str(session),
               "board": str(args.board_root), "profile": str(args.profile),
               "port": args.port, "device": args.device, "explicit_program": args.program})
    controller = QualifiedController(args.board_root, args.profile, session, args.port, args.work_dir)
    console = RotationConsole(controller, run_dir, args.device)
    console.numeric(0.0, "startup_real0")
    show_help()
    while True:
        try:
            text = input("angle/correct> ")
            command, angle = parse_command(text)
            if command == "quit":
                print("已退出；板卡保留最后应用的配置。")
                return 0
            if command == "angle":
                console.numeric(float(angle))
            elif command == "correct":
                console.correct()
            elif command == "status":
                console.status()
            elif command == "help":
                show_help()
        except (EOFError, KeyboardInterrupt):
            print("\n控制台退出。若刚才中断了配置操作，请重新连接核验当前状态。")
            return 0
        except Exception as exc:
            print(f"操作未完成：{exc}\n记录目录：{run_dir}", flush=True)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n已取消。")
        raise SystemExit(130)
    except Exception as exc:
        print(f"启动失败：{exc}\n请检查UART/板卡供电；断电后请重新下载合格位流。", file=sys.stderr)
        raise SystemExit(1)
