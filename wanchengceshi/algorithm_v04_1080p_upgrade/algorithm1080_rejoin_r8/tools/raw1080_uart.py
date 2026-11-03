"""RAW8-specific diagnostic interpretation and qualified-session sampling."""
if not __debug__:
    raise RuntimeError('Qualified hardware tools require Python assertions enabled; do not use -O or PYTHONOPTIMIZE')
from pathlib import Path
import argparse,datetime,hashlib,json,sys,time
board=Path(__file__).resolve().parents[1];root=board.parent
sys.path.insert(0,str(root.parent/'tools'))
from decode_algorithm_packet import decode_packet
def decode_raw1080(blob):
    d=decode_packet(blob);w=d['words']
    if d['storageDiagnosticVersion']!=6 or not d['readbackComplete'] or d['readbackMismatch']:
        raise ValueError('Wrong/stale FPGA mode or SCCB readback failure')
    if (w[8]>>8)&255 or (w[8]>>16)&255!=3:
        raise ValueError('Not expected OV5640 RAW output4300=00,501f=03')
    if (w[24]&0x70)!=0x70:raise ValueError('Camera/DDR/pixel diagnostic snapshot is stale')
    if (d['configuredWidth'],d['configuredHeight'])!=(1920,1080):raise ValueError('Sensor configured size is not1080')
    if (d['rawLineMinBytes'],d['rawLineMaxBytes'],d['rawFrameLines'])!=(1920,1920,1080):
        raise ValueError('Observed RAW8 line/frame geometry is not1920x1080')
    if w[29]!=1920*1080:raise ValueError('HDMI DE pixels per frame are not2073600')
    d.update(legacyRgb565WidthMinPixels=d['rawLineMinBytes']//2,legacyRgb565WidthMaxPixels=d['rawLineMaxBytes']//2,
        rawWidthMinPixels=d['rawLineMinBytes'],rawWidthMaxPixels=d['rawLineMaxBytes'],
        interpretation='RAW8 one byte/pixel; legacy RGB565 capture/normalizer counters are diagnostic-only',
        hdmiPixelsPerFrame=w[29],hdmiVsyncCount=w[31])
    d.update(sourceAbortCauseCounters=dict(zip(['notReady','short','excess','lateGrant'],[(w[28]>>i)&255 for i in [0,8,16,24]])),
        rawCoreProtocolErrorCount=w[33],fetchRequestBusyCycles=None,fetchRequestAcceptedCount=None,
        diagnosticWordOverrides='R8 W28 four independent modular8 source abort counters; W33 RAW core protocol count, not old fetch gauges')
    counts=dict(zip(['packetMismatch','unexpectedSof','dirtyEnd','incompleteEnd'],[(w[34]>>i)&255 for i in [0,8,16,24]]))
    detail=w[35]
    d.update(rawAbortBranchCounters=counts,
        rawAbortLastDetail=dict(inputPixelTag=(detail>>8)&0x3fffff,reservedHigh2=detail>>30,
            packetMismatch=bool(detail&1),unexpectedSof=bool(detail&2),dirtyEnd=bool(detail&4),
            incompleteEnd=bool(detail&8),previousPendingEnd=bool(detail&16),dropFrame=bool(detail&32),
            droppedSamples=bool(detail&64),captureChanged=bool(detail&128),endFlagsValid=bool(detail&12)),
        diagnosticWordOverrides='R8 W28 source aborts,W33 RAW protocol,W34 RAW branch counters,W35 last abort detail; W34/W35 are not AXI read gauges')
    return d

def raw_branch_deltas(first,last):
    camera=(last['cameraFrameCount']-first['cameraFrameCount'])&0xffffffff
    protocol=(last['rawCoreProtocolErrorCount']-first['rawCoreProtocolErrorCount'])&0xffffffff
    if not 0<camera<256 or protocol>=256:
        raise ValueError('Ambiguous 8-bit counter interval: camera or RAW protocol bound failed')
    delta={k:(last['rawAbortBranchCounters'][k]-first['rawAbortBranchCounters'][k])&255 for k in first['rawAbortBranchCounters']}
    if any(v>protocol for v in delta.values()):
        raise ValueError('RAW branch increments exceed original protocol aggregate')
    if delta['packetMismatch']+delta['unexpectedSof']+max(delta['dirtyEnd'],delta['incompleteEnd'])>protocol:
        raise ValueError('RAW mutually exclusive branch lower bound exceeds protocol aggregate')
    return delta

def main():
    import serial
    stream=root.parent/'algorithm_v04_performance_stream720_20261014'
    sys.path.insert(0,str(stream/'tools'))
    from measurement_guard import measure
    from program_session_guard import check_session
    p=argparse.ArgumentParser();p.add_argument('--acceptance',type=Path,required=True)
    p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--port',default='COM14');p.add_argument('--seconds',type=float,default=24);a=p.parse_args()
    assert 8<=a.seconds<=120 and not a.output.exists()
    acceptance=json.loads(a.acceptance.read_text());session=json.loads(a.session.read_text())
    assert acceptance['passed'] and acceptance['candidate']=='algorithm1080_rejoin_r8';check_session(session,acceptance)
    for name,digest in {**acceptance['sourceHashes'],**acceptance['evidenceHashes']}.items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
    a.output.mkdir(parents=True)
    result=dict(scope='Real new native1080 geometry candidate: distinct camera/source/result counters, RAW-END diagnosis and SDA6 configuration-applied observations; not color-chart proof',
        port=a.port,programSession=session,acceptance=str(a.acceptance),snapshots=[],
        startedUtc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    uart=serial.Serial(port=None,baudrate=115200,timeout=.1,write_timeout=2)
    uart.port=a.port;uart.dtr=False;uart.rts=False
    try:
        uart.open();start=time.monotonic();target=start
        while True:
            time.sleep(max(0,target-time.monotonic()));uart.reset_input_buffer();stamp=time.monotonic();uart.write(b'\xa5');buf=bytearray();packet=None
            while time.monotonic()-stamp<4:
                buf+=uart.read(512);off=buf.find(b'\x55\xaa\xd1\xf4')
                if off>=0 and len(buf)>=off+251:packet=bytes(buf[off:off+251]);break
            if packet is None:raise TimeoutError('No complete FPGA diagnostic response')
            # Preserve mode/geometry evidence even when RAW validation rejects it.
            result['lastReceivedHex']=packet.hex()
            result['lastDecodedBeforeRawValidation']=decode_packet(packet)
            decoded=decode_raw1080(packet)
            result['snapshots'].append(dict(requestSeconds=stamp-start,hex=packet.hex(),decoded=decoded))
            print(f"t={stamp-start:.3f} source={decoded['sourceFrameCommitCount']} result={decoded['resultFrameCommitCount']} raw={decoded['rawWidthMinPixels']}x{decoded['rawFrameLines']}",flush=True)
            if stamp-start>=a.seconds:break
            target+=4
        duration,deltas,fps=measure(result['snapshots'])
        hdmi=0;pclk=0
        for first,last in zip(result['snapshots'],result['snapshots'][1:]):
            hdmi+=(last['decoded']['hdmiVsyncCount']-first['decoded']['hdmiVsyncCount'])&0xffffffff
            pclk+=(last['decoded']['cameraPclkCount']-first['decoded']['cameraPclkCount'])&0xffffffff
        intervals=[]
        for first,last in zip(result['snapshots'],result['snapshots'][1:]):
            intervals.append(dict(startSeconds=first['requestSeconds'],endSeconds=last['requestSeconds'],
                rawBranchDeltas=raw_branch_deltas(first['decoded'],last['decoded']),
                rawProtocolDelta=(last['decoded']['rawCoreProtocolErrorCount']-first['decoded']['rawCoreProtocolErrorCount'])&0xffffffff,
                lastAbortDetail=last['decoded']['rawAbortLastDetail']))
        result['rawAbortIntervals']=intervals
        result.update(status='CAPTURE_COMPLETE',measuredSeconds=duration,counterDeltas=deltas,measuredFps=fps,
            hdmiVsyncFps=hdmi/duration,cameraPclkHz=pclk/duration,
            algorithmErrorLow6=[s['decoded']['algorithmErrorLow6'] for s in result['snapshots']])
    except Exception as exc:result.update(status='FAILED',error=repr(exc));raise
    finally:
        uart.close();result['endedUtc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k in ['status','measuredFps','hdmiVsyncFps','counterDeltas','error']}),flush=True)
if __name__=='__main__':main()
