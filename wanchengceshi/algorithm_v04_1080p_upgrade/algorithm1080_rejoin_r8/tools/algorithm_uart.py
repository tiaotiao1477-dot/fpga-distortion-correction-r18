"""Native1080 v0.4 payload preparation and qualified SRAM-session control.

prepare performs the original full-range integer audit; send requires its
unmodified payload/report and the new candidate's qualified hardware session.
"""
if not __debug__:
    raise RuntimeError('Qualified hardware tools require Python assertions enabled; do not use -O or PYTHONOPTIMIZE')
from pathlib import Path
import argparse,datetime,hashlib,json,sys,time
b=Path(__file__).resolve().parents[1];u=b.parent;base=u.parent/'algorithm_v04_integration';delivery=base/'delivery/distortion_algorithm_delivery_v0_4'
sys.path[:0]=[str(delivery),str(base/'tools'),str(u.parent/'algorithm_v04_performance_stream720_20261014/tools')]
from make_uart_packet import packet
from decode_algorithm_packet import decode_packet
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def validate_native_payload(payload):
 if len(payload)!=92:raise ValueError('Expected complete92byte v0.4 payload')
 words=[int.from_bytes(payload[i:i+4],'little') for i in range(0,92,4)]
 if tuple(words[i] for i in [0,1,18,19])!=(1920,1080,1920,1080):raise ValueError('Only native1920x1080 source/output accepted')
 return words
def application_confirmed(before,after,sequence):
 if not before['fresh'] or not after['fresh']:return False
 if sequence in (before['receivedSequence'],before['appliedSequence']):return False
 return (after['receivedSequence']==sequence and after['appliedSequence']==sequence and
    0<((after['receivedCount']-before['receivedCount'])&0xffffffff)<0x80000000 and
    0<((after['appliedCount']-before['appliedCount'])&0xffffffff)<0x80000000 and
    0<((after['activeVersion']-before['activeVersion'])&65535)<32768)
def query(uart):
 uart.reset_input_buffer();uart.write(b'\xa5');data=bytearray();stamp=time.monotonic();off=-1
 while time.monotonic()-stamp<2:
  data+=uart.read(512);off=data.find(b'\x55\xaa\xd1\xf4')
  if off>=0 and len(data)>=off+251:break
 if off<0 or len(data)<off+251:raise TimeoutError('No complete new diagnostic response')
 original=bytes(data[off:off+251]);d=decode_packet(original)
 if (d['words'][24]&0x70)!=0x70 or not d['readbackComplete'] or d['readbackMismatch']:raise ValueError('Stale/invalid camera/DDR/pixel snapshots')
 if (d['configuredWidth'],d['configuredHeight'])!=(1920,1080):raise ValueError('Non-native sensor mode')
 return original,d
def main():
 p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
 q=sub.add_parser('prepare');q.add_argument('--profile',type=Path,default=b/'native_identity_profile.json');q.add_argument('--angle',type=float,required=True)
 q.add_argument('--border',choices=['black','clamp'],default='clamp');q.add_argument('--mode',choices=['original','magnitude','binary','overlay'],default='original')
 q.add_argument('--threshold',type=int,default=200);q.add_argument('--output',type=Path,required=True)
 q=sub.add_parser('send');q.add_argument('--config',type=Path,required=True);q.add_argument('--sequence',type=int,required=True)
 q.add_argument('--acceptance',type=Path,required=True);q.add_argument('--session',type=Path,required=True);q.add_argument('--output',type=Path,required=True);q.add_argument('--port',default='COM14')
 a=p.parse_args()
 if a.command=='prepare':
  from sw.reference_common.camera_profile import load_camera_profile
  from sw.reference_common.geometry import RemapConfig,InverseAffine
  from sw.reference_common.edge_processing import EdgeConfig,BorderMode
  from sw.host.generate_edge_config import export,MODE_NAMES
  profile=load_camera_profile(a.profile)
  if (profile.width,profile.height)!=(1920,1080):raise ValueError('A real native1080 profile is required;720p profile is not scaled automatically')
  cfg=EdgeConfig(RemapConfig(profile,InverseAffine.rotation(a.angle,1920,1080)),BorderMode.CLAMP if a.border=='clamp' else BorderMode.BLACK,a.threshold,MODE_NAMES[a.mode])
  assert not a.output.exists();export(cfg,a.output,max_width=1920)
  validate_native_payload((a.output/'config_payload.bin').read_bytes())
  files=[a.profile,a.output/'config_payload.bin',a.output/'config.json',a.output/'range_report.json',Path(__file__)]
  (a.output/'binding.json').write_text(json.dumps({str(p.resolve()):sha(p) for p in files},indent=2)+'\n')
  print('Native payload/range audit complete:',a.output);return
 import serial
 from program_session_guard import check_session
 acceptance=json.loads(a.acceptance.read_text());session=json.loads(a.session.read_text())
 assert acceptance['passed'] and acceptance['candidate']=='algorithm1080_rejoin_r8';check_session(session,acceptance)
 for path,h in {**acceptance['sourceHashes'],**acceptance['evidenceHashes'],**json.loads((a.config/'binding.json').read_text())}.items():assert sha(path)==h,path
 payload=(a.config/'config_payload.bin').read_bytes();validate_native_payload(payload);wire=packet(payload,a.sequence)
 assert not a.output.exists();a.output.mkdir(parents=True);(a.output/'sent_packet.bin').write_bytes(wire)
 r=dict(sequence=a.sequence,programSession=session,config=str(a.config),configBindingSha256=sha(a.config/'binding.json'),
    acceptanceSha256=sha(a.acceptance),sentPacketSha256=sha(a.output/'sent_packet.bin'),snapshots=[],status='STARTED')
 uart=serial.Serial(port=None,baudrate=115200,timeout=.1,write_timeout=2);uart.port=a.port;uart.dtr=False;uart.rts=False
 try:
  uart.open();original,baseline=query(uart);r['baseline']=dict(hex=original.hex(),decoded=baseline)
  before=baseline['algorithmConfig']
  if a.sequence in (before['receivedSequence'],before['appliedSequence']):raise ValueError('Choose a new sequence, not an already observed sequence')
  uart.write(wire);uart.flush();start=time.monotonic()
  while time.monotonic()-start<15:
   original,d=query(uart);r['snapshots'].append(dict(hex=original.hex(),decoded=d))
   state=d['algorithmConfig']
   if application_confirmed(before,state,a.sequence):
    r.update(status='APPLIED',applied=state);break
   time.sleep(.25)
  else:raise TimeoutError('Configuration not observed applied to a source frame')
 except Exception as e:r.update(status='FAILED',error=repr(e));raise
 finally:
  uart.close();r['endedUtc']=datetime.datetime.now(datetime.timezone.utc).isoformat();(a.output/'result.json').write_text(json.dumps(r,indent=2)+'\n')
 print('Applied:',r['applied'])
if __name__=='__main__':main()
