"""Program a fully qualified candidate into SRAM and bind actual readback."""
if not __debug__:
    raise RuntimeError('Qualified hardware tools require Python assertions enabled; do not use -O or PYTHONOPTIMIZE')
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
assert not a.output.exists()
acceptance = json.loads((root / 'reports/physical_acceptance.json').read_text())
assert acceptance['passed']
for path, expected in {**acceptance['evidenceHashes'], **acceptance['sourceHashes']}.items():
    assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
bit = Path(acceptance['bitstream']).resolve()
assert bit.suffix == '.sbit'
a.output.mkdir(parents=True)
tcl = a.output / 'program.tcl'
tcl.write_text('cfg_set_tcl_break -flag true\n'
    'cfg_scan_chain\n'
    'cfg_read_device_property -mode 0 -device_index 0\n'
    f'cfg_assign_file -file "{bit.as_posix()}" -device_index 0\n'
    'cfg_program -device_index 0\n'
    'cfg_verify -device_index 0\n'
    'cfg_read_device_property -mode 3 -device_index 0\n'
    'cfg_disconnect\n', encoding='utf-8')
started = a.output / 'started_utc.txt'
started.write_text(datetime.now(timezone.utc).isoformat(), encoding='utf-8')
log = a.output / 'program.log'
with log.open('w', encoding='utf-8') as stream:
    process = subprocess.run(['D:/FPGA/fpga1/PDS_2022.2-SP6.4/bin/cdt_cfg_shell.exe',
        '-ip','127.0.0.1','-port','65422','-file',str(tcl.resolve())],
        cwd=a.output, stdout=stream, stderr=subprocess.STDOUT, timeout=300)
assert process.returncode == 0
# CLI exit status alone cannot establish readback success; require bound log
# success, correct device and post-verify DONE through the existing guard.
subprocess.run([sys.executable, '-B', str(root / 'tools/create_program_session.py'),
    '--log',str(log.resolve()),'--started',str(started.resolve()),
    '--output',str((a.output / 'program_session.json').resolve())], check=True)
