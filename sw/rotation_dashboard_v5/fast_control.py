"""V3 controller retaining the qualified R8 session, send and ACK transaction."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import uuid

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'rotation_console_v1'))
from qualified_control import (QualifiedController, BIT_SHA256, PROFILE_SHA256,
                               HELPER_SHA256, _sha, _load)

from fast_geometry import (BIT_SHA, PROFILE_SHA, HELPER_SHA, affine_for,
                           reference_hashes, prepare_config)
from sw.reference_common.geometry import RemapConfig
from sw.reference_common.edge_processing import EdgeConfig, BorderMode, edge_config_words
from sw.reference_common.camera_profile import load_camera_profile
from sw.reference_fixed.combined_distortion import CONFIG_WORD_NAMES


class FastQualifiedController(QualifiedController):
    """Drop-in controller with persistent exact-angle V3 full-raster configs."""

    def __init__(self, *args, progress=None, **kwargs):
        self.crop_enabled = False
        self.crop_applied: dict | None = None
        self._prepared_config: Path | None = None
        self.progress = progress
        super().__init__(*args, **kwargs)

    def _log_run(self, label, argv, timeout=360):
        command = list(argv)
        if (label == 'send' and len(command) >= 4 and command[3] == 'send' and
                Path(command[2]).resolve() == self._helper.resolve()):
            command[2] = str(HERE / 'fast_sender.py')
        return super()._log_run(label, command, timeout)

    def crop_state(self) -> dict:
        known = self.current_angle is not None and self.last_applied is not None
        return {'known': known, 'enabled': self.crop_enabled if known else None,
                'zoom': self.crop_applied.get('zoom', 1.0) if known and self.crop_applied else (1.0 if known else None),
                'report': str(self._prepared_config / 'crop_report.json')
                          if known and self.crop_enabled and self._prepared_config else None,
                'note': 'V3已审计完整帧和四采样点；画质/吞吐仍需板测'
                        if known and self.crop_enabled else '未裁剪，保持原始全视野和CLAMP边界策略'}

    def snapshot(self) -> dict:
        return {**super().snapshot(), 'hostCropState': self.crop_state()}

    def set_crop(self, enabled: bool) -> dict:
        self.require_unchanged()
        angle = self.current_angle
        prior = self.crop_enabled
        self.crop_enabled = bool(enabled)
        try:
            return self.apply_angle(angle)
        except BaseException:
            self.crop_enabled = prior
            self.crop_applied = None
            raise

    def apply_angle(self, angle: float) -> dict:
        try:
            # Base class preserves pre-send fresh state, frozen acceptance/session,
            # original CLI source SHA check, sequence ACK, and post-send readback.
            result = super().apply_angle(angle)
            self.crop_applied = (_load(self._prepared_config / 'crop_report.json')
                                 if self.crop_enabled and self._prepared_config else None)
            return {**result, 'hostCropState': self.crop_state()}
        except BaseException:
            self.crop_applied = None
            self.current_angle = None
            self.last_applied = None
            raise

    def _cache_key(self, angle: float) -> str:
        refs = reference_hashes()
        modules = {str(p.resolve()): _sha(p) for p in
                   (HERE / 'fast_geometry.py', HERE / 'fast_audit.py', HERE / 'fast_control.py',
                   HERE / 'content_hash_batch.py', HERE / 'fast_sender.py')}
        material = {'angleHex': float(angle).hex(), 'cropEnabled': self.crop_enabled,
                    'bitSha256': BIT_SHA, 'profileSha256': PROFILE_SHA,
                    'sendHelperSha256': HELPER_SHA, 'references': refs, 'v3Modules': modules}
        return hashlib.sha256(json.dumps(material, sort_keys=True, separators=(',', ':')).encode()).hexdigest()[:32]

    def _config_dir(self, angle: float) -> Path:
        key = float(angle) % 360.0
        cache = HERE / 'config_cache'
        cache.mkdir(exist_ok=True)
        digest = self._cache_key(key)
        for candidate in sorted(cache.glob(digest + '_*')):
            if self._valid_config(candidate, key):
                self._prepared_config = candidate
                return candidate
        target = cache / f'{digest}_{uuid.uuid4().hex}'
        callback = None
        if self.progress is not None:
            callback = lambda done, total: self.progress(
                f'V3全帧定点审计 {done:,}/{total:,} 像素')
        prepared = prepare_config(self.profile_path, self._helper, key, target,
                                  crop=self.crop_enabled, progress=callback)
        if not self._valid_config(prepared, key):
            raise ValueError('V3 config failed exact binding/profile/full-frame validation')
        qualified = _load(prepared / 'qualified_binding.json')
        qualified.update({'cacheKey': digest,
                          'bindingSha256': _sha(prepared / 'binding.json')})
        (prepared / 'qualified_binding.json').write_text(
            json.dumps(qualified, indent=2) + '\n', encoding='utf-8')
        self._prepared_config = prepared
        return prepared

    def _valid_config(self, folder: Path, angle: float) -> bool:
        try:
            if not folder.is_dir():
                return False
            binding = _load(folder / 'binding.json')
            if not isinstance(binding, dict) or not binding:
                return False
            for name, digest in binding.items():
                if _sha(Path(name)) != digest:
                    return False
            refs = reference_hashes()
            if any(binding.get(path) != digest for path, digest in refs.items()):
                return False
            required = [self.profile_path, self._helper, HERE / 'fast_geometry.py',
                        HERE / 'fast_audit.py', HERE / 'fast_control.py',
                        HERE / 'content_hash_batch.py', HERE / 'fast_sender.py',
                        *(folder / name for name in
                          ('config.json', 'config_payload.bin', 'range_report.json', 'crop_report.json'))]
            if any(binding.get(str(path.resolve())) != _sha(path) for path in required):
                return False
            if (_sha(self.profile_path) != PROFILE_SHA or _sha(self._helper) != HELPER_SHA or
                    BIT_SHA256 != BIT_SHA or PROFILE_SHA256 != PROFILE_SHA or HELPER_SHA256 != HELPER_SHA):
                return False
            cfg, crop, report = (_load(folder / 'config.json'), _load(folder / 'crop_report.json'),
                                 _load(folder / 'range_report.json'))
            if (cfg.get('crop_enabled') is not self.crop_enabled or
                    crop.get('cropEnabled') is not self.crop_enabled or
                    crop.get('passed') is not True or crop.get('count') != 1920 * 1080 or
                    crop.get('profileSha256') != PROFILE_SHA or crop.get('bitSha256') != BIT_SHA or
                    crop.get('sendHelperSha256') != HELPER_SHA or
                    crop.get('generatorSha256') != _sha(HERE / 'fast_geometry.py') or
                    crop.get('auditModuleSha256') != _sha(HERE / 'fast_audit.py')):
                return False
            if (report.get('full_frame_widths_fit') is not True or report.get('count') != 1920 * 1080 or
                    report.get('step') != 1 or report.get('affine_overflow_count') != 0 or
                    report.get('coordinate_saturation_count') != 0 or
                    not report.get('fixed_ranges') or
                    not all(value.get('fits_rtl_width') is True for value in report['fixed_ranges'].values())):
                return False
            if self.crop_enabled and (crop.get('outsideCount') != 0 or crop.get('invalidFetch4Count') != 0 or
                                      crop.get('allPixelsAndFourTapsInside') is not True):
                return False
            if abs(math.remainder(float(crop['angle']) - angle, 360)) > 1e-12:
                return False
            scale = float(crop['inverseScale'])
            if not math.isfinite(scale) or not 0 < scale <= 1 or abs(float(crop['zoom']) * scale - 1) > 1e-12:
                return False
            if not self.crop_enabled and abs(scale - 1.0) > 1e-12:
                return False
            profile = load_camera_profile(self.profile_path)
            expected_affine = affine_for(angle, scale)
            expected = EdgeConfig(RemapConfig(profile, expected_affine), BorderMode.CLAMP, 200, 0)
            expected_words = edge_config_words(expected)
            payload = (folder / 'config_payload.bin').read_bytes()
            if len(payload) != 92:
                return False
            binary = tuple(int.from_bytes(payload[i:i + 4], 'little') for i in range(0, 92, 4))
            if binary != tuple(int(word) & 0xFFFFFFFF for word in expected_words):
                return False
            names = (*CONFIG_WORD_NAMES, 'border_mode', 'edge_threshold', 'edge_mode')
            if (cfg.get('profile') != profile.to_mapping() or cfg.get('inverse_affine') != expected_affine.__dict__ or
                    cfg.get('output_width') != 1920 or cfg.get('output_height') != 1080 or
                    cfg.get('words') != dict(zip(names, expected_words))):
                return False
            qualified = _load(folder / 'qualified_binding.json')
            return (qualified.get('bitSha256') == BIT_SHA and
                    qualified.get('profileSha256') == PROFILE_SHA and
                    qualified.get('sendHelperSha256') == HELPER_SHA and
                    qualified.get('bindingSha256') == _sha(folder / 'binding.json'))
        except (OSError, KeyError, TypeError, ValueError, OverflowError, json.JSONDecodeError):
            return False
