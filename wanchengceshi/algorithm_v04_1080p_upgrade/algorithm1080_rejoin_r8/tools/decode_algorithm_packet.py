"""SDA6 original-byte validation and native RAW/algorithm observations.

The frozen legacy decoder validates framing/XOR and decodes common fields.
No fabricated marker, recomputed checksum or modified packet is used.
"""
if not __debug__:
    raise RuntimeError('Qualified hardware tools require Python assertions enabled; do not use -O or PYTHONOPTIMIZE')
from pathlib import Path
import importlib.util
_path=Path(__file__).resolve().parents[3]/'tools/decode_camera_mode.py'
_spec=importlib.util.spec_from_file_location('legacy_camera_mode_sda6',_path)
_legacy=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_legacy)
MARKER=int.from_bytes(b'SDA6','little')
def decode_packet(packet):
    d=_legacy.decode_packet(packet) # validates ORIGINAL CRC, length, header/trailer
    w=d['words']
    if w[15]!=MARKER:raise ValueError('SDA6 required; legacy candidate is not this algorithm build')
    d.update(storageDiagnosticVersion=6,ddrWriteLastCount=w[25],sourceFrameCommitCount=w[26],
        resultFrameCommitCount=w[37],sourceFrameDropCount=w[39],sourceFifoOverflowCount=w[40]>>2,
        ddrReadCommandReady=bool(w[40]&1),ddrReadCommandDone=bool(w[40]&2),
        sourceWriterState=w[36]&15,sourceWriterAllocReady=bool(w[36]&16),sourceWriterFrameOwned=bool(w[36]&32),
        sourceSlotStates=[(w[36]>>(6+i*3))&7 for i in range(3)],
        algorithmState=w[38]&15,algorithmX=(w[38]>>4)&2047,algorithmY=(w[38]>>15)&2047,
        algorithmErrorLow6=w[38]>>26,resultWriterStallCount=w[30],
        rawWidthMinPixels=w[12]&65535,rawWidthMaxPixels=w[12]>>16,
        rawProtocolErrors=w[33],rawAbortBranches=w[34],rawAbortDetail=w[35],
        sourceAbortCauses=dict(zip(['notReady','short','excess','lateGrant'],[(w[28]>>(8*i))&255 for i in range(4)])))
    # Legacy fetch-request fields do not describe this RAW diagnostic schema.
    d.pop('fetchRequestBusyCycles',None);d.pop('fetchRequestAcceptedCount',None)
    d['algorithmConfig']=dict(valid=bool(w[42]&1),ready=bool(w[42]&2),frameError=bool(w[42]&4),resultIdle=bool(w[42]&8),
        receivedSequence=w[43],appliedSequence=w[44],appliedSourceFrame=w[45],activeVersion=w[46]&65535,
        receivedCount=w[47],rejectedCount=w[48],appliedCount=w[49],fresh=bool(w[24]&(1<<5)))
    return d
