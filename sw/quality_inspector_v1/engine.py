"""Serialized test orchestration, persistent independent per-item outcomes."""
from pathlib import Path
import csv
import json
import math
import time
import uuid
from datetime import datetime,timezone

from .analysis import summarize_performance,summarize_gamma,summarize_cnn,parse_resources
from .image_analysis import analyze_session

KEYS=('geometry','sharpness','performance','edges','gamma','cnn','resources','latency')
TITLES=dict(zip(KEYS,('校正精度','清晰度','帧率与掉帧','边缘与视野','Gamma','CNN','资源与时序','硬件延迟')))


def utc():return datetime.now(timezone.utc).isoformat()


def make_row(key,status='not_measured',value='未测',detail='',source='live'):
    return dict(key=key,title=TITLES[key],status=status,value=value,detail=detail,metrics={},evidence=[],limits=[],source=source)


class InspectorEngine:
    def __init__(self,root,adapter,analyzer=analyze_session,notify=None):
        self.root=Path(root).resolve();self.root.mkdir(parents=True,exist_ok=True)
        self.adapter=adapter;self.analyzer=analyzer;self.notify=notify or (lambda *a:None)
        self.connected=False
        self.session=dict(id=uuid.uuid4().hex,createdUtc=utc(),source='live',
            results={k:make_row(k) for k in KEYS},artifacts=[],notes=[
                '测量完成不等于官方合格；没有官方数字门槛的指标只报告实测值。',
                '图像对比会使用真实标定0°；同源像素链零畸变系数基线不是传感器RAW。',
                '连接不会下载FPGA、加载CNN模型或修改几何参数。'])
        self.folder=self.root/self.session['id'];self.folder.mkdir()
        self.persist()

    def persist(self):
        path=self.folder/'session.json';temp=path.with_suffix('.tmp')
        temp.write_text(json.dumps(self.session,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');temp.replace(path)

    def update(self,key,result):
        value=dict(make_row(key),**result);value['key']=key;value['title']=TITLES[key];value['updatedUtc']=utc()
        if value['source']=='live' and value['status']=='measured':value.setdefault('measuredUtc',value['updatedUtc'])
        self.session['results'][key]=value;self.persist();self.notify('item',(key,value))

    def add_capture_artifacts(self,path):
        """Bundle original frames and configs, not just the summary pictures."""
        path=Path(path);self.session['artifacts'].append(str(path.resolve()))
        self.session['artifacts'].extend(str(p.resolve()) for p in path.parent.rglob('*.json') if p.is_file())
        raw=json.loads(path.read_text(encoding='utf-8'))
        for stage in raw.get('stages',{}).values():
            self.session['artifacts'].extend(stage.get('images',[]))
            if stage.get('captureRecord'):self.session['artifacts'].append(stage['captureRecord'])
            folder=Path(stage.get('configDir',''))
            if stage.get('configDir') and folder.is_dir():
                self.session['artifacts'].extend(str(p) for p in folder.rglob('*') if p.is_file() and p.suffix in ('.json','.bin','.hex'))
        self.session['artifacts']=list(dict.fromkeys(self.session['artifacts']))

    def connect(self,port,device):
        if self.connected:return
        try:
            context=self.adapter.connect(port,device,self.folder/'connection')
        except Exception:
            self.adapter.close();raise
        self.connected=True;self.session['connection']=context;self.persist()
        self.notify('connected',(True,context.get('detail','已连接；连接仅执行诊断')))

    def disconnect(self):
        self.adapter.close();self.connected=False;self.notify('connected',(False,'已断开；已有结果仍可导出'))

    def measure(self,keys,duration,stop,latency_target=300):
        keys=list(dict.fromkeys(k for k in keys if k in KEYS));run=self.folder/('test_'+uuid.uuid4().hex[:10]);run.mkdir()
        if not keys:return
        for key in keys:self.update(key,make_row(key,'running','测量中'))
        image_keys=[k for k in keys if k in ('geometry','sharpness','edges')]
        if image_keys:
            if not self.connected:
                for key in image_keys:self.update(key,make_row(key,'condition_required','请先连接','需要真实板卡与HDMI图像'))
            elif stop.is_set():
                for key in image_keys:self.update(key,make_row(key,'cancelled','已取消'))
            else:
                try:
                    capture=self.adapter.capture_pair(run/'capture',stop,lambda message:self.notify('log',message))
                    if Path(capture).is_file():self.add_capture_artifacts(capture)
                    if stop.is_set():
                        for key in image_keys:self.update(key,make_row(key,'cancelled','已停止；配置恢复结果已记录'))
                    else:
                        self.notify('log','三组原图已保存，正在生成校正前后对照和各项数值；无需手动切换或拍照。')
                        outcomes=self.analyzer(capture,run/'analysis',image_keys)
                        for key in image_keys:
                            result=outcomes.get(key,make_row(key,'error','分析未返回结果'))
                            result['source']='live';self.update(key,result)
                except Exception as exc:
                    if (run/'capture/session.json').is_file():self.add_capture_artifacts(run/'capture/session.json')
                    for key in image_keys:self.update(key,make_row(key,'cancelled' if stop.is_set() else 'condition_required',
                        '已停止' if stop.is_set() else '需要检查测量条件',str(exc)))
        observe_keys=[k for k in keys if k in ('performance','gamma','cnn')]
        if observe_keys:
            if not self.connected:
                for key in observe_keys:self.update(key,make_row(key,'condition_required','请先连接','没有真实板卡观测'))
            elif stop.is_set():
                for key in observe_keys:self.update(key,make_row(key,'cancelled','已取消'))
            else:
                observations=[];errors=[];started=time.monotonic()
                try:
                    while True:
                        try:observations.append(self.adapter.sample())
                        except Exception as exc:errors.append(str(exc))
                        elapsed=time.monotonic()-started
                        self.notify('log',f'遥测采样 {elapsed:.0f}/{duration:.0f}秒，{len(observations)}个有效样本')
                        if stop.is_set() or elapsed>=duration:break
                        stop.wait(min(1.,max(0.,duration-elapsed)))
                    p=run/'telemetry.json';p.write_text(json.dumps({'samples':observations,'errors':errors},ensure_ascii=False,indent=2),encoding='utf-8')
                    self.session['artifacts'].append(str(p))
                    reducers={'performance':lambda:summarize_performance(observations),
                        'gamma':lambda:summarize_gamma([v['capabilities']['a8']['decoded'] for v in observations if (v.get('capabilities',{}).get('a8') or {}).get('decoded')]),
                        'cnn':lambda:summarize_cnn([v['capabilities']['a7'] for v in observations if v.get('capabilities',{}).get('a7')])}
                    for key in observe_keys:
                        if stop.is_set():result=make_row(key,'cancelled','已停止','部分原始遥测已保留')
                        else:
                            try:result=reducers[key]()
                            except Exception as exc:result=make_row(key,'error','分析失败',str(exc))
                        result.setdefault('evidence',[]).append(str(p));self.update(key,result)
                except Exception as exc:
                    for key in observe_keys:self.update(key,make_row(key,'error','采样失败',str(exc)))
        if 'latency' in keys:
            if stop.is_set():self.update('latency',make_row('latency','cancelled','已取消'))
            elif not self.connected:self.update('latency',make_row('latency','condition_required','请先连接','R16硬件时间戳测量需要真实板卡；也可导入外部CSV。'))
            else:
                try:
                    result=self.adapter.measure_latency(run/'latency',stop,lambda m:self.notify('log',m),target=latency_target)
                    self.session['artifacts'].extend(str(p.resolve()) for p in (run/'latency').rglob('*') if p.is_file())
                    self.update('latency',result)
                except Exception as exc:
                    self.session['artifacts'].extend(str(p.resolve()) for p in (run/'latency').rglob('*') if p.is_file())
                    self.update('latency',make_row('latency','cancelled' if stop.is_set() else 'condition_required',
                        '已停止' if stop.is_set() else '延迟测量未完成',str(exc)))
        for key in [k for k in keys if k=='resources']:
            previous=self.session.get('imports',{}).get(key)
            self.update(key,previous or make_row(key,'condition_required','需要导入报告' if key=='resources' else '需要实测延迟CSV',
                '点击本项“导入”；一键测量不伪造缺失的外部证据。',source='imported'))
        self.persist();self.notify('session',self.session)

    def import_resources(self,paths):
        result=parse_resources(paths);self.session.setdefault('imports',{})['resources']=result;self.update('resources',result)

    def import_latency(self,path):
        p=Path(path)
        with p.open(encoding='utf-8-sig',newline='') as stream:
            reader=csv.DictReader(stream)
            if not reader.fieldnames or not {'latency_ms','method'}.issubset(reader.fieldnames):
                raise ValueError('CSV需要latency_ms和method两列，method填写真实仪器/测量方法。')
            rows=list(reader)
        if not rows or not all(row.get('method','').strip() for row in rows):
            raise ValueError('CSV需要latency_ms和method两列，method填写真实仪器/测量方法。')
        vals=[float(row['latency_ms']) for row in rows]
        if not all(math.isfinite(v) and v>=0 for v in vals):raise ValueError('延迟必须是有限非负毫秒数')
        result=make_row('latency','measured',f'{sum(vals)/len(vals):.3f} ms 平均','来自外部测量CSV；不是电脑预览或指令ACK延迟。','imported')
        result['metrics']={'samples':len(vals),'mean_ms':sum(vals)/len(vals),'minimum_ms':min(vals),'maximum_ms':max(vals),'method':list(dict.fromkeys(r['method'] for r in rows))}
        result['evidence']=[str(p.resolve())];result['limits']=['导入值的仪器精度和同步方法需在比赛报告中说明。']
        self.session.setdefault('imports',{})['latency']=result;self.update('latency',result)

    def import_history(self,path,keys=None):
        path=Path(path).resolve();raw=json.loads(path.read_text(encoding='utf-8'))
        if 'stages' in raw:
            out=self.folder/('history_'+uuid.uuid4().hex[:8])
            result=self.analyzer(path,out,keys or ['geometry','sharpness','edges'])
            self.add_capture_artifacts(path)
            try:
                record=json.loads(Path(raw['stages']['baseline1']['captureRecord']).read_text(encoding='utf-8'))
                original_utc=record.get('startedUtc')
                for value in result.values():value['measuredUtc']=original_utc
            except (OSError,KeyError,ValueError):pass
        elif isinstance(raw.get('results'),dict):
            # Exported self-contained packages must verify their copied files.
            for record in raw.get('sourceIntegrity',[]):
                copied=record.get('copiedPath')
                if copied:
                    import hashlib
                    p=(path.parent/copied).resolve()
                    if not p.is_relative_to(path.parent) or not p.is_file():raise ValueError('报告证据路径无效')
                    expected=record.get('sha256') or record.get('copiedSha256')
                    if not expected or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:raise ValueError('报告证据SHA256不匹配')
            result=raw['results']
            for r in result.values():
                paths=[]
                for value in r.get('evidence',[]):
                    p=Path(value)
                    if not p.is_absolute():
                        relative=(path.parent/p).resolve()
                        legacy=(Path(__file__).resolve().parents[2]/p).resolve()
                        p=relative if relative.is_file() else legacy
                    paths.append(str(p))
                r['evidence']=paths
        else:raise ValueError('请选择本软件session.json或三组采集session.json')
        for key,r in result.items():
            if key in KEYS:r['source']='historical';self.update(key,r)
        self.session['notes'].append('已导入历史记录：'+str(path));self.persist();self.notify('session',self.session)
