"""Hardware adapter: one owner, explicit measurement writes, cooperative restore."""
from pathlib import Path
import sys
import time
import json
import hashlib

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'sw/rotation_dashboard_v5'),str(ROOT)]
from interactive_control import LiveBoardController,DEFAULT_PROFILE,_live_read_a5
from preview_stream import PreviewStream
from auto_correction import _validated_transaction
from tools.sfr_measurement.measurement_config import prepare_geometry
from .pair_policy import check_status, gamma_assessment


def save(p,obj):Path(p).write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


class MeasurementCancelled(RuntimeError):pass


class LiveAdapter:
    def __init__(self):self.controller=None;self.preview=None;self.port=None

    def connect(self,port,device,output):
        self.port=port;output=Path(output);output.mkdir(parents=True,exist_ok=True)
        controller=LiveBoardController(port,DEFAULT_PROFILE,output/'control',lambda _:None)
        connection=controller.connect()
        self.controller=controller;self.preview=PreviewStream(device,output/'preview');self.preview.start()
        return dict(detail='已连接；接口协议核对，不代表位流回读认证',port=port,device=device,connection=connection,
                    profilePath=str(DEFAULT_PROFILE),profileSha256=hashlib.sha256(DEFAULT_PROFILE.read_bytes()).hexdigest())

    def close(self):
        if self.preview is not None:self.preview.stop()
        self.preview=None;self.controller=None

    def sample(self,include_caps=True):
        start=time.monotonic();a5=_live_read_a5(self.port);stamp=(start+time.monotonic())/2
        caps={}
        if include_caps:
            try:
                caps=self.controller._fresh_capabilities()
                a7=caps.get('a7') or {}
                if a7.get('rawHex'):
                    from monitor_model import decode_cnn
                    from interactive_control import NN_TOOLS
                    full=decode_cnn(bytes.fromhex(a7['rawHex']),NN_TOOLS/'nn_host.py')
                    a7['predictionCycles']=full.get('cycles');a7['nnCycles']=full.get('nn_cycles')
            except Exception as exc:caps={'error':str(exc)}
        return {'monotonic':stamp,'a5':a5,'capabilities':caps}

    def capture_pair(self,output,stop,notify):
        output=Path(output);output.mkdir(parents=True,exist_ok=False)
        c=self.controller
        result={'scope':'live_protocol_only_paired_output_domain_SFR','sourceResolution':[1920,1080],
            'sourceEncoding':'HDMI MJPEG30 decoded to PNG, not sensor RAW','profilePath':str(DEFAULT_PROFILE),
            'stages':{},'restoredMeasured0':False,'programmedBoard':False,'modelWrites':False,
            'boardIdentity':'live interface marker only; no SRAM readback','source':'live',
            'gammaPolicy':'Observe each stage; no identity-bank requirement and no Gamma writes',
            'automaticSequence':['baseline1','calibrated','baseline2'],
            'baselineMeaning':'Same HDMI chain with zero distortion coefficients; not sensor RAW'}
        attempted=False;measured=None;gamma_lease=None
        def record():save(output/'session.json',result)
        def cancelled():
            if stop.is_set():raise MeasurementCancelled('用户已停止，正在完成安全恢复')
        try:
            cancelled();result['initial']=check_status(c);record()
            if (result['initial'].get('capabilities',{}).get('a7') or {}).get('capabilityWord')==0x000f0007:
                from .measurement_live import GammaLease
                gamma_lease=GammaLease(self.port,output/'gamma_control')
                result['gammaOverride']={'initial':None,'applied':gamma_lease.acquire_identity()}
                result['gammaOverride']['initial']=gamma_lease.initial
                result['gammaPolicy']='R16 actual frame-boundary identity override with acknowledged ownership'
                record();notify('已固定恒等Gamma；等待新完整帧后自动采集三组。')
            notify('自动保存：校正前原图 → 真实参数校正图 → 重复校正前；结束保留真实参数0°。')
            measured=prepare_geometry(c,'measured0',output/'configs/measured0')
            cancelled();identity=prepare_geometry(c,'identity_baseline',output/'configs/identity_baseline')
            for label,config in (('baseline1',identity),('calibrated',measured),('baseline2',identity)):
                cancelled();notify({'baseline1':'采集校正前','calibrated':'采集真实参数校正后','baseline2':'复测基线稳定性'}[label])
                before=check_status(c);attempted=True
                transaction=_validated_transaction(config,c,c.binding_path,c.connection_path,self.port,
                    output/('transaction_'+label),binding_validator=c.validate_context)
                save(output/('transaction_'+label)/'result.json',transaction)
                baseline=transaction['ack_diagnostic']['resultFrameCommitCount'];deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    cancelled();state=c.require_unchanged();delta=(state['resultFrameCommitCount']-baseline)&0xffffffff
                    if 8<=delta<256:break
                    stop.wait(.1)
                else:raise RuntimeError('新参数已ACK，但结果帧未继续提交')
                cancelled();before_capture=check_status(c)
                if gamma_lease is not None:
                    measurement_before=gamma_lease.check()
                    deadline=time.monotonic()+3
                    while True:
                        g=(before_capture['capabilities'].get('a8') or {}).get('decoded') or {}
                        if g.get('valid') is True and g.get('valid_count')==2073600 and g.get('applied_bank')==2 and g.get('next_bank')==2:break
                        if time.monotonic()>=deadline:raise RuntimeError('Gamma固定已ACK，但尚未取得新的完整恒等帧统计')
                        cancelled();stop.wait(.08);before_capture=check_status(c)
                batch=self.preview.collect(output/label)
                after_capture=check_status(c)
                b,e=before_capture['a5']['algorithmConfig'],after_capture['a5']['algorithmConfig']
                if any(b[k]!=e[k] for k in ('receivedSequence','appliedSequence','activeVersion')):
                    raise RuntimeError('采集中板卡配置发生外部变化，结果不可比较')
                delta=(after_capture['a5']['resultFrameCommitCount']-before_capture['a5']['resultFrameCommitCount'])&0xffffffff
                if not 1<=delta<256:raise RuntimeError('采集期间没有新的结果帧')
                measurement_after=gamma_lease.check() if gamma_lease is not None else None
                result['stages'][label]={'configDir':str(config),'images':list(map(str,batch.image_paths)),
                    'measurementBefore':measurement_before if gamma_lease is not None else None,'measurementAfter':measurement_after,
                    'captureRecord':str(batch.record_path),'beforeTransaction':before,'beforeCapture':before_capture,
                    'afterCapture':after_capture,'resultCommitsDuringCapture':delta}
                record()
            result['gammaAssessment']=gamma_assessment(result);record()
            notify(result['gammaAssessment']['detail'])
        except Exception as exc:
            result['error']=str(exc);record();raise
        finally:
            if attempted and measured is not None:
                try:
                    if not c.last_applied:raise RuntimeError('ACK所有权未知，未盲目恢复，请重新连接确认板卡')
                    notify('恢复真实标定0°，请等待事务结束。')
                    restored=_validated_transaction(measured,c,c.binding_path,c.connection_path,self.port,
                        output/'transaction_restore',binding_validator=c.validate_context)
                    save(output/'transaction_restore/result.json',restored)
                    result['restoredMeasured0']=restored['status']=='APPLIED';result['restorationACK']=restored['applied']
                except Exception as exc:result['restorationError']=str(exc)
            if gamma_lease is not None:
                try:result.setdefault('gammaOverride',{})['restored']=gamma_lease.restore()
                except Exception as exc:result.setdefault('gammaOverride',{})['restorationError']=str(exc)
            result['sourceToolHashes']={str(Path(__file__).resolve()):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
            record()
        if not result['restoredMeasured0']:raise RuntimeError('恢复未确认：'+result.get('restorationError','未知'))
        if result.get('gammaOverride',{}).get('restorationError'):
            raise RuntimeError('图像已保存，但Gamma恢复未确认：'+result['gammaOverride']['restorationError'])
        return output/'session.json'

    def measure_latency(self,output,stop,notify,target=300):
        from .measurement_live import SerialLease,latency_result
        from .latency_benchmark import run_latency_benchmark
        from .latency_protocol import decode_packet
        output=Path(output);output.mkdir(parents=True,exist_ok=False)
        c=self.controller;initial=check_status(c)
        if initial['capabilities']['a7'].get('capabilityWord')!=0x000f0007:
            raise RuntimeError('硬件时间戳需要新R16工程；当前版本仍可导入外部延迟CSV，不能用UART往返时间替代。')
        link=SerialLease(self.port);packets=output/'all_a9_packets.bin'
        def save_packet(packet):
            with packets.open('ab') as stream:stream.write(packet)
        def angle_reader(_):
            link.release();state=c.require_unchanged()
            if c.current_angle is None:raise RuntimeError('实际角度未知，未开始延迟统计')
            return {'angle_q16':round(c.current_angle*65536),'cfg_version':state['algorithmConfig']['activeVersion']}
        def angle_setter(q16):
            link.release()
            notify(f'正在应用并确认 {q16/65536:g}°；随后预热30帧、采集{target}个对应源帧。')
            applied=c.apply_angle(q16/65536)
            if applied.get('status')!='APPLIED':raise RuntimeError('角度未获真实板卡ACK')
            return angle_reader(link)
        try:
            # Explicit documented safe end-state; never infer unknown boot geometry.
            angle_setter(0)
            result=run_latency_benchmark(link,capability_reader=lambda _:initial['capabilities']['a7']['capabilityWord'],
                angle_reader=angle_reader,angle_setter=angle_setter,target=target,cancel_event=stop,
                timeout_per_angle=max(60,5*(target+30)/30),raw_packet_sink=save_packet)
            return latency_result(result,output/'analysis')
        except Exception as exc:
            save(output/'failure.json',{'error':str(exc),'notes':getattr(exc,'__notes__',[]),
                'rawPacketsSaved':packets.is_file(),'hardwareTimestampResult':False})
            raise
        finally:link.release()
