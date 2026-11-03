"""Live paired e-SFR acquisition. No programming, models, or FPGA edits."""
from pathlib import Path
import sys, argparse, json, time, uuid, hashlib

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'sw/rotation_dashboard_v5'), str(ROOT)]
from interactive_control import LiveBoardController, DEFAULT_PROFILE
from preview_stream import PreviewStream
from auto_correction import _validated_transaction
from tools.sfr_measurement.measurement_config import prepare_geometry
from tools.sfr_measurement import detect_grid, candidate_rois


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def check_status(controller):
    a5 = controller.require_unchanged()
    caps = controller._fresh_capabilities()
    nn = caps.get('a7') or {}
    gamma = (caps.get('a8') or {}).get('decoded') or {}
    if nn.get('enabled') is not False or nn.get('capabilityWord') != 0x000f0003:
        raise RuntimeError('Need live R15 marker and CNN disabled for paired SFR')
    if not gamma.get('valid') or gamma.get('valid_count') != 1920*1080:
        raise RuntimeError('Display Gamma statistics are not a complete 1080p frame')
    if gamma.get('applied_bank') != 2 or gamma.get('next_bank') != 2:
        raise RuntimeError('Gamma is not stable identity bank2; paired result would be confounded')
    return {'a5': a5, 'capabilities': caps}


def settle(controller, result_count):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        state = controller.require_unchanged()
        delta = (state['resultFrameCommitCount'] - result_count) & 0xffffffff
        if 8 <= delta < 256:
            time.sleep(.15)
            return
        time.sleep(.1)
    raise RuntimeError('Result commits did not advance after geometry ACK')


def run(port, output, device, check_only):
    output.mkdir(parents=True, exist_ok=False)
    controller = LiveBoardController(port, DEFAULT_PROFILE, output/'control', lambda _: None)
    preview = PreviewStream(device, output/'preview')
    result = {'scope': 'live_protocol_only_paired_output_domain_SFR', 'output': str(output),
              'programmedBoard': False, 'modelWrites': False, 'sourceResolution': [1920,1080],
              'sourceEncoding': 'HDMI capture MJPEG30 decoded to individual PNGs; not sensor RAW',
              'boardIdentity': 'user-reported R15 plus A7 interface marker; no new SRAM readback',
              'checkOnly': check_only, 'stages': {}, 'restoredMeasured0': False}
    measured = None
    changed = False
    def record(): save(output/'session.json', result)
    try:
        result['connection'] = controller.connect()
        result['initial'] = check_status(controller)
        record()
        preview.start()
        if check_only:
            batch = preview.collect(output/'pose_check')
            grid = detect_grid(batch.frames[2])
            candidates = candidate_rois(grid, batch.frames[2].shape)
            result['stages']['pose_check'] = {'images': list(map(str,batch.image_paths)),
                'corners': grid.tolist(), 'candidates': candidates}
            result['final'] = check_status(controller)
            record()
            print(json.dumps({'session': str(output/'session.json'),
                              'image': str(batch.image_paths[2]), 'candidates': len(candidates)}), flush=True)
            return
        measured = prepare_geometry(controller, 'measured0', output/'configs/measured0')
        identity = prepare_geometry(controller, 'identity_baseline', output/'configs/identity_baseline')
        # A full-frame preparation binds every stage/product and current live context.
        for label, config in (('baseline1', identity), ('calibrated', measured), ('baseline2', identity)):
            before = check_status(controller)
            transaction = _validated_transaction(config, controller, controller.binding_path,
                controller.connection_path, port, output/('transaction_'+label),
                binding_validator=controller.validate_context)
            changed = True
            save(output/('transaction_'+label)/'result.json', transaction)
            settle(controller, transaction['ack_diagnostic']['resultFrameCommitCount'])
            capture_before = check_status(controller)
            batch = preview.collect(output/label)
            capture_after = check_status(controller)
            delta = (capture_after['a5']['resultFrameCommitCount']-
                     capture_before['a5']['resultFrameCommitCount']) & 0xffffffff
            if not 1 <= delta < 256:
                raise RuntimeError('No new result commits during capture')
            for key in ('receivedSequence','appliedSequence','activeVersion'):
                if capture_before['a5']['algorithmConfig'][key] != capture_after['a5']['algorithmConfig'][key]:
                    raise RuntimeError('Geometry ACK changed during capture')
            # Corner detection is annotation, not an acquisition/ACK gate.
            # A partially cropped chart can still contain valid local slanted edges.
            grid, grid_error = None, None
            try:
                grid = detect_grid(batch.frames[2])
            except ValueError as exc:
                grid_error = str(exc)
            result['stages'][label] = {'configDir': str(config), 'images': list(map(str,batch.image_paths)),
                'captureRecord': str(batch.record_path), 'beforeTransaction': before,
                'beforeCapture': capture_before, 'afterCapture': capture_after,
                'transactionResult': str(output/('transaction_'+label)/'result.json'),
                'corners': grid.tolist() if grid is not None else None,
                'cornerDetectionError': grid_error, 'resultCommitsDuringCapture': delta,
                'SFRQualified': False}
            record()
            print(json.dumps({'captured': label, 'images': len(batch.image_paths),
                              'resultCommitsDuringCapture': delta}), flush=True)
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        record()
        raise
    finally:
        if changed and measured is not None:
            try:
                if not controller.last_applied:
                    raise RuntimeError('ACK ownership unknown; no blind restoration attempted')
                restored = _validated_transaction(measured, controller, controller.binding_path,
                    controller.connection_path, port, output/'transaction_restore_measured0',
                    binding_validator=controller.validate_context)
                save(output/'transaction_restore_measured0/result.json', restored)
                result['restoredMeasured0'] = restored['status'] == 'APPLIED'
                result['restorationACK'] = restored['applied']
            except Exception as exc:
                result['restorationError'] = f'{type(exc).__name__}: {exc}'
        preview.stop()
        result['previewStopped'] = preview.status()
        result['sourceToolHashes'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(__file__), ROOT/'tools/sfr_measurement/measurement_config.py')}
        record()
    print(str(output/'session.json'), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM14')
    parser.add_argument('--device', default='USB Video')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute: parser.error('Explicit --execute required for UART/capture')
    run(args.port, args.output.resolve(), args.device, args.check_only)
