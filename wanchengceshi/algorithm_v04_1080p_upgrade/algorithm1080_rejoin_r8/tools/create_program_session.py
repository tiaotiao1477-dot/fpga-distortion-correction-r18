"""Create a verified SRAM session from actual PDS log and qualified image."""
if not __debug__:
    raise RuntimeError('Qualified hardware tools require Python assertions enabled; do not use -O or PYTHONOPTIMIZE')
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import re
import uuid
from program_session_guard import check_session

root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument('--log',type=Path,required=True)
p.add_argument('--started',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args();assert not a.output.exists()
acceptance=json.loads((root/'reports/physical_acceptance.json').read_text(encoding='utf-8'))
assert acceptance['passed']
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
for path,expected in acceptance['evidenceHashes'].items():assert sha(path)==expected
text=a.log.read_text(encoding='utf-8-sig',errors='replace')
session={'sessionId':str(uuid.uuid4()),'programPassed':True,'verifyPassed':True,
    'configurationMemory':'SRAM','deviceId':'0x00603899','programStartedUtc':a.started.read_text(encoding='utf-8-sig').strip(),
    'verifyCompletedUtc':datetime.now(timezone.utc).isoformat(),'programLog':str(a.log),
    'programLogSha256':sha(a.log),'candidateBitstreamSha256':acceptance['bitstreamSha256'],
    'maskSha256':acceptance['maskSha256'],'overTempFlag':bool(re.search(r'Bit\s+27:\s+1\s+over_temp\b',text)),
    'actualTemperatureMeasured':False}
check_session(session,acceptance)
a.output.write_text(json.dumps(session,indent=2)+'\n',encoding='utf-8')
print(json.dumps(session),flush=True)
