"""Wrap a qualified 92-byte v0.4 payload; this utility does not open a serial port."""
from pathlib import Path
import argparse,binascii
def packet(payload:bytes,sequence:int)->bytes:
    if len(payload)!=92:raise ValueError('v0.4 payload must have 92 bytes')
    if not 0<=sequence<=0xffffffff:raise ValueError('sequence out of uint32 range')
    body=bytes([4,92])+sequence.to_bytes(4,'little')+payload
    return bytes.fromhex('c33c')+body+binascii.crc_hqx(body,0xffff).to_bytes(2,'little')
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--payload',required=True,type=Path)
    p.add_argument('--sequence',required=True,type=int)
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    data=packet(a.payload.read_bytes(),a.sequence)
    with a.output.open('xb') as f:f.write(data)
    print(f'Generated {len(data)} bytes; no UART transmission performed')
