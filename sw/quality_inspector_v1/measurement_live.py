"""R16 services used only inside the inspector's serialized hardware worker."""
from pathlib import Path
import json
import struct
import time

from .measurement_transport import MeasurementCommands,MeasurementTransport

R16_MARKER=0x000f0007

class SerialLease:
    """Lazily borrow the worker-owned port; release before legacy controller calls."""
    def __init__(self,port,factory=None):
        self.port=port;self.uart=None;self.factory=factory
    def _open(self):
        if self.uart is None:
            if self.factory is None:
                import serial
                factory=serial.Serial
            else:factory=self.factory
            self.uart=factory(port=None,baudrate=115200,timeout=.02,write_timeout=.5)
            self.uart.dtr=False;self.uart.rts=False;self.uart.port=self.port;self.uart.open()
        return self.uart
    def write(self,data):return self._open().write(data)
    def read(self,size):return self._open().read(size)
    def release(self):
        uart,self.uart=self.uart,None
        if uart is not None:uart.close()

class GammaLease:
    """Explicit override + ACK ownership, retaining every original A9 packet."""
    def __init__(self,port,output,link=None):
        self.link=link or SerialLease(port);self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.records=[];self.initial=None;self.owned_seq=None
        self.commands=MeasurementCommands(MeasurementTransport(self.link),packet_sink=self._record)
    def _record(self,packet):
        from .latency_protocol import decode_packet
        self.records.append(decode_packet(packet))
        with (self.output/'a9_packets.bin').open('ab') as stream:stream.write(packet)
        (self.output/'a9_decoded.json').write_text(json.dumps(self.records,indent=2),encoding='utf-8')
    def current(self):
        try:return self.commands.transport_record()
        finally:self.link.release()
    def acquire_identity(self):
        self.initial=self.current()
        if self.initial['gamma_busy']:raise RuntimeError('Gamma 正在切换，未改写模式，请稍后重测。')
        try:
            applied=self.commands.set_gamma_identity(True,previous=self.initial)
            self.owned_seq=applied['gamma_cmd_applied_seq']
            return applied
        finally:self.link.release()
    def check(self):
        value=self.current()
        if (self.owned_seq is None or value['gamma_cmd_applied_seq']!=self.owned_seq
            or not value['gamma_identity'] or value['gamma_busy']):
            raise RuntimeError('测量期间 Gamma 固定控制发生变化，停止对比；不覆盖外部设置。')
        return value
    def restore(self):
        if self.owned_seq is None:return None
        current=self.check()
        try:return self.commands.set_gamma_identity(self.initial['gamma_identity'],previous=current)
        finally:self.link.release()

def latency_result(raw,folder):
    """Save hardware-derived tables and curves; never infer latency from UART timing."""
    import csv
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    packets=raw.pop('raw_packets',[])
    (folder/'a9_packets.bin').write_bytes(b''.join(packets))
    (folder/'summary.json').write_text(json.dumps(raw,ensure_ascii=False,indent=2),encoding='utf-8')
    figure,ax=plt.subplots(figsize=(9,4))
    means=[];maxima=[];p95s=[]
    with (folder/'samples.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.writer(stream);writer.writerow(['angle_deg','source_id','sensor_id','T0','TinputLast','T1','T2','latency_ms'])
        for angle,summary in raw['per_angle'].items():
            stats=summary['latency_ms']['t2_minus_t0'];means.append(stats['avg']);maxima.append(stats['max']);p95s.append(stats['p95'])
            samples=summary['samples'];ax.plot([s['source_frame_id'] for s in samples],
                [s['latency_ms']['t2_minus_t0'] for s in samples],label=f'{int(angle)/65536:g} deg',linewidth=.8)
            for s in samples:writer.writerow([int(angle)/65536,s['source_frame_id'],s['sensor_frame_id'],
                s['t0_ticks'],s['t_input_last_ticks'],s['t1_ticks'],s['t2_ticks'],s['latency_ms']['t2_minus_t0']])
    ax.set(xlabel='Source frame ID',ylabel='T2 - T0 (ms)',title='FPGA input to parallel HDMI first pixel')
    ax.legend();ax.grid(alpha=.3);figure.tight_layout();figure.savefig(folder/'latency_curves.png',dpi=150);plt.close(figure)
    return dict(status='measured',value=f'平均 {sum(means)/len(means):.3f} ms · P95最高 {max(p95s):.3f} ms',
        detail='0°、15°、45°、90°分组；平均值为各组等样本均值，P95/最大值逐组保留。',
        metrics=raw,evidence=[str(p.resolve()) for p in folder.iterdir() if p.is_file()],
        limits=['T0为DVP首有效像素观察，T2为FPGA并行HDMI首像素；不含曝光、HDMI发射芯片和显示器。',
                '125MHz计时量化8ns；跨时钟同步和输入流水偏移须计入不确定度，不代表8ns绝对精度。'])
