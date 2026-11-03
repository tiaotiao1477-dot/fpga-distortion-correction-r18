"""Isolated launcher that delegates qualified sends to the pinned R8 helper."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import types

from content_hash_batch import HashBatch

BOARD_ROOT = Path(__file__).resolve().parents[2]
HELPER_PATH = (BOARD_ROOT / 'wanchengceshi' / 'algorithm_v04_1080p_upgrade' /
               'algorithm1080_rejoin_r8' / 'tools' / 'algorithm_uart.py').resolve()
HELPER_SHA256 = '64a6941b0ffaee1068a92fef0b133f9712bfab57db4fadfe7e999c9d5f46fa45'


def load_qualified_helper(path: Path = HELPER_PATH):
    path = Path(path).resolve(strict=True)
    source = path.read_bytes()
    actual = hashlib.sha256(source).hexdigest()
    if actual != HELPER_SHA256:
        raise RuntimeError(f'Frozen R8 sender SHA-256 mismatch: {path}')
    spec = importlib.util.spec_from_file_location('_qualified_algorithm_uart_r8', path)
    if spec is None:
        raise ImportError(f'Unable to load frozen R8 sender: {path}')
    module = types.ModuleType(spec.name)
    module.__file__ = str(path)
    module.__spec__ = spec
    # Python normally places a script's tools directory at sys.path[0];
    # preserve that when the frozen script is imported by this launcher.
    sys.path.insert(0, str(path.parent))
    exec(compile(source, str(path), 'exec'), module.__dict__)
    return module


def _option(argv: list[str], name: str) -> Path:
    for index in range(len(argv) - 2, -1, -1):
        if argv[index] == name:
            return Path(argv[index + 1])
    raise ValueError(f'Missing required sender option {name}')


def collect_hash_paths(argv: list[str]) -> tuple[list[Path], Path]:
    if len(argv) < 2 or argv[1] != 'send':
        raise ValueError('The optimized adapter only accepts the qualified send command')
    acceptance_path = _option(argv, '--acceptance')
    config_path = _option(argv, '--config')
    acceptance = json.loads(acceptance_path.read_text(encoding='utf-8'))
    binding_path = config_path / 'binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    paths = {}
    for section in ('sourceHashes', 'evidenceHashes'):
        values = acceptance.get(section)
        if not isinstance(values, dict):
            raise ValueError(f'Acceptance manifest has no valid {section}')
        for value in values:
            path = str(Path(value).resolve(strict=True))
            paths[path] = Path(path)
    if not isinstance(binding, dict) or not binding:
        raise ValueError('Config binding must be a nonempty path-to-SHA-256 mapping')
    for value in binding:
        path = str(Path(value).resolve(strict=True))
        paths[path] = Path(path)
    # The original helper also hashes these two manifests for its output record.
    paths[str(acceptance_path.resolve(strict=True))] = acceptance_path.resolve(strict=True)
    paths[str(binding_path.resolve(strict=True))] = binding_path.resolve(strict=True)
    return list(paths.values()), _option(argv, '--output')


def run_helper(helper_module, argv: list[str], hash_paths, output: Path | None = None):
    """Run the unchanged helper with only sha accelerated for this invocation."""
    legacy_sha = helper_module.sha
    prior_argv = sys.argv
    batch = HashBatch(hash_paths)

    def transaction_sha(path):
        try:
            return batch.sha(path)
        except KeyError:
            # The original helper hashes its newly-created small sent-packet file too.
            return legacy_sha(path)

    helper_module.sha = transaction_sha
    sys.argv = list(argv)
    try:
        return helper_module.main()
    finally:
        helper_module.sha = legacy_sha
        sys.argv = prior_argv
        if output is not None and output.is_dir():
            report = {'schemaVersion': 1, 'qualifiedHelperPath': str(HELPER_PATH),
                      'qualifiedHelperSha256': HELPER_SHA256,
                      'hashBatch': batch.report()}
            try:
                (output / 'hash_batch_report.json').write_text(
                    json.dumps(report, indent=2) + '\n', encoding='utf-8')
            except OSError:
                pass
        batch.close()


def main() -> None:
    helper = load_qualified_helper()
    argv = [str(HELPER_PATH), *sys.argv[1:]]
    paths, output = collect_hash_paths(argv)
    run_helper(helper, argv, paths, output)


if __name__ == '__main__':
    main()
