"""Owned hidden CDT server and frozen byte-identical qualified SRAM helper."""
if not __debug__:
    raise RuntimeError('Qualified hardware tools require Python assertions enabled; do not use -O or PYTHONOPTIMIZE')
from pathlib import Path
import argparse,hashlib,json,re,socket,subprocess,sys,time
b=Path(__file__).resolve().parents[1];u=b.parent
p=argparse.ArgumentParser();p.add_argument('--attempt',default='program');a=p.parse_args();assert re.fullmatch(r'program(?:_retry[1-9][0-9]*)?',a.attempt)
out=u/'reports/algorithm1080_board_rejoin_r8_20261022';target=out/a.attempt
acceptance=json.loads((b/'reports/physical_acceptance.json').read_text());assert acceptance['passed'] and acceptance['candidate']==b.name
for p,h in {**acceptance['sourceHashes'],**acceptance['evidenceHashes']}.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
from serial.tools import list_ports
ports=[dict(device=p.device,vid=p.vid,pid=p.pid,description=p.description) for p in list_ports.comports()]
assert any(x['vid']==0x0403 and x['pid']==0x6010 for x in ports),'Qualified JTAG USB device not enumerated'
def listening():
 with socket.socket() as sock:sock.settimeout(.2);return sock.connect_ex(('127.0.0.1',65422))==0
assert not listening(),'Existing CDT service; inspect ownership before proceeding'
assert not target.exists(),'Keep previous attempt; select a new explicit retry directory'
out.mkdir(exist_ok=True);(out/(a.attempt+'_enumeration.json')).write_text(json.dumps(ports,indent=2)+'\n')
work=out/(a.attempt+'_cdt_server');work.mkdir(exist_ok=False)
with (out/(a.attempt+'_cdt_server.log')).open('x') as log:
 server=subprocess.Popen(['D:/FPGA/fpga1/PDS_2022.2-SP6.4/bin/cdt_js.exe','-port','65422','-work_dir',str(work)],
 stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
 try:
  deadline=time.monotonic()+15
  while not listening():
   assert server.poll() is None and time.monotonic()<deadline,'Owned CDT readiness failure';time.sleep(.1)
  subprocess.run([sys.executable,'-X','utf8','-B',str(b/'tools/program_qualified_candidate.py'),'--output',str(target)],check=True,timeout=300)
 finally:
  if server.poll() is None:
   server.terminate()
   try:server.wait(timeout=5)
   except subprocess.TimeoutExpired:server.kill();server.wait(timeout=5)
print('New qualified SRAM/Verify/DONE session recorded; only self-owned CDT service closed')
