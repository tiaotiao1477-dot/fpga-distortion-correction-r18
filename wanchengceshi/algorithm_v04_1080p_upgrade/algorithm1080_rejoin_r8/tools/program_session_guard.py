"""Bind board measurements to completed SRAM program/readback evidence."""
if not __debug__:
    raise RuntimeError('Qualified hardware tools require Python assertions enabled; do not use -O or PYTHONOPTIMIZE')
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import re

def check_session(session,acceptance,now=None):
    now = now or datetime.now(timezone.utc)
    bit = Path(acceptance['bitstream'])
    sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    assert session['programPassed'] is True and session['verifyPassed'] is True
    assert session['deviceId'].lower()=='0x00603899'
    assert session['configurationMemory']=='SRAM'
    assert sha(bit)==session['candidateBitstreamSha256']==acceptance['bitstreamSha256']
    assert sha(acceptance['mask'])==session['maskSha256']==acceptance['maskSha256']
    start = datetime.fromisoformat(session['programStartedUtc'])
    end = datetime.fromisoformat(session['verifyCompletedUtc'])
    assert start.tzinfo is not None and end.tzinfo is not None
    assert start<=end<=now, 'Unfinished or future programming evidence'
    log_path = Path(session['programLog'])
    assert sha(log_path)==session['programLogSha256'], 'Programming log changed'
    log = log_path.read_text(encoding='utf-8-sig',errors='replace')
    assert not any(line.startswith('E:') for line in log.splitlines()), 'Programming/readback error'
    for command in ['cfg_program','cfg_verify']:
        assert f'Executing : {command} -device_index 0 successfully.' in log
    assert 'The done bit is 1' in log
    assert f'cfg_assign_file -file {bit.as_posix()} -device_index 0' in log
    assert 'The ID value is: 0x00603899' in log
    after_verify=log.split('Executing : cfg_verify -device_index 0 successfully.')[-1]
    assert re.search(r'Bit\s+12:\s+1\s+done\b',after_verify), 'DONE lost after verify'
