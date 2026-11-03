"""Pure, observation-only summaries for quality inspector inputs."""
from pathlib import Path
import hashlib
import re


def _row(key, status, title, value, detail, metrics=None, evidence=None, limits=None, source='live'):
    return dict(key=key, status=status, title=title, value=value, detail=detail,
                metrics=metrics or {}, evidence=evidence or [], limits=limits or [], source=source)


def _u32_delta(a, b):
    if b >= a:
        return b - a
    if a > 0xF0000000 and b < 0x0FFFFFFF:
        return (1 << 32) - a + b
    return None


def summarize_performance(samples):
    names = ('cameraFrameCount', 'sourceFrameCommitCount', 'resultFrameCommitCount', 'sourceFrameDropCount')
    if len(samples) < 2:
        return _row('performance','condition_required','性能','需要至少两个样本','样本不足',limits=['计数器必须来自新帧遥测'])
    a, b = samples[0], samples[-1]
    dt = float(b.get('monotonic', 0)) - float(a.get('monotonic', 0))
    try:
        deltas = {n: _u32_delta(int(a['a5'][n]), int(b['a5'][n])) for n in names}
        for first,second in zip(samples,samples[1:]):
            elapsed=float(second['monotonic'])-float(first['monotonic'])
            if elapsed<=0 or any(_u32_delta(int(first['a5'][n]),int(second['a5'][n])) is None for n in names):
                raise ValueError('counter reset within observation window')
            # Protect against a malformed counter packet being mistaken for FPS.
            if any(_u32_delta(int(first['a5'][n]),int(second['a5'][n]))/elapsed>240 for n in names[:3]):
                raise ValueError('counter jumps beyond supported camera rates')
    except (KeyError, TypeError, ValueError):
        deltas = {}
    if dt <= 0 or len(deltas) != len(names) or any(v is None for v in deltas.values()):
        return _row('performance','condition_required','性能','计数器无效','时间无效、字段缺失或计数器发生非回绕复位',{'elapsed_seconds':dt},limits=['复位区间不计算帧率'])
    metrics = {'elapsed_seconds':dt, **{k: v for k,v in deltas.items()},
               'camera_fps':deltas[names[0]]/dt,'source_commit_fps':deltas[names[1]]/dt,
               'result_commit_fps':deltas[names[2]]/dt,'source_drop_fps':deltas[names[3]]/dt}
    return _row('performance','measured','帧率与掉帧',f"结果 {metrics['result_commit_fps']:.2f} FPS · 丢帧 {deltas[names[3]]}",
        f"相机{metrics['camera_fps']:.2f} / 源{metrics['source_commit_fps']:.2f} FPS，窗口{dt:.1f}秒；依据真实A5计数差分。",metrics,
        limits=['非官方通过判定；预览FPS不能代替结果提交帧率。']+(['结果帧没有进展，请检查板内处理。'] if not deltas[names[2]] else []))


def summarize_gamma(observations):
    if not observations:
        return _row('gamma','condition_required','Gamma','无观测','未提供Gamma观测')
    ids = [o.get('frame_id') for o in observations]
    valid=all(o.get('valid') is True and o.get('valid_count')==2073600 for o in observations)
    if not valid or len(ids) < 2 or ids[-1] == ids[0] or None in ids or _u32_delta(ids[0],ids[-1]) is None:
        return _row('gamma','condition_required','Gamma','帧未更新','frame_id缺失或未变化，观测可能过期',{'observations':observations[-2:]},limits=['只报告寄存器观测，不推断图像增强效果'])
    last=observations[-1];bank=last.get('applied_bank');mean=last.get('mean_luminance')
    return _row('gamma','measured','Gamma',f"档位 {bank if bank is not None else '未知'} · 均值{mean:.1f}" if mean is not None else '统计已更新',
        '来自实际显示域完整帧统计；档位5为混合处理，配置版本65535表示未知。',
        {'observations':observations,'frameIncrement':_u32_delta(ids[0],ids[-1])},limits=['不代表增强精度或画质改善；强光/暗场对照需固定场景。'])


def summarize_cnn(observations):
    if not observations:
        return _row('cnn','condition_required','CNN','无观测','未提供CNN状态')
    o = observations[-1]
    if 'enabled' not in o and 'loaded' not in o:
        return _row('cnn','condition_required','CNN','状态未知','缺少CNN能力字段',{'observation':o})
    state = '已禁用' if not o.get('enabled', False) else '已启用'
    n=o.get('predictionCount');cycles=o.get('predictionCycles')
    faults=any(o.get(k) for k in ('nnError','protocolError','faultLatched','saturated'))
    return _row('cnn','measured','CNN',state+(' · '+str(n)+'次预测' if n is not None else ''),
        '仅状态观测，不自动加载或启用；'+('模型未加载。' if not o.get('loaded') else '模型已加载。')+('板端报告错误。' if faults else ''),
        {'observation':o,'predictionCycles':cycles,'accuracyMeasured':False},limits=['预测次数、耗时不代表准确率，准确率需要对应参考真值。'])


def parse_resources(paths):
    files=[];resources={};timing=[];performance=[]
    for path in paths:
        p=Path(path)
        rec={'path':str(p),'sha256':None,'format':p.suffix.lower(),'status':'error'}
        try:
            raw=p.read_bytes(); text=raw.decode('utf-8',errors='replace')
            rec['sha256']=hashlib.sha256(raw).hexdigest()
            rec['design']= (re.search(r'^\s*\|?\s*Design\s*:\s*(.+)$',text,re.M) or [None,''])[1].strip()
            rec['device']= (re.search(r'^\s*\|?\s*Device\s*:\s*(.+)$',text,re.M) or [None,''])[1].strip()
            slack=re.search(r'^\s*Slack\s*\((MET|VIOLATED)\)\s+([-+\d.]+)',text,re.M)
            if slack: rec['slack_ns']=float(slack.group(2)); rec['slack_status']=slack.group(1)
            if p.suffix.lower()=='.rur' and 'Device Utilization Summary' in text:
                local={}
                for label,used,available in re.findall(r'^\|\s*([^|]+?)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|',text,re.M):
                    label=label.strip();key={'+ LUT':'LUT','+ FF':'FF','+ APM':'APM','DRM(grid)':'DRM_grid'}.get(label)
                    if key:
                        value=local.setdefault(key,{'used':0,'available':0})
                        value['used']+=int(used);value['available']+=int(available)
                for value in local.values():value['percent']=100*value['used']/value['available'] if value['available'] else None
                if local:resources.update(local);rec['resources']=local;rec['status']='parsed'
            elif p.suffix.lower()=='.rtr' and 'Performance Summary' in text:
                blocks=re.split(r'(?m)^((?:Setup|Hold|Recovery|Removal|Minimum Pulse Width) Summary\((?:Slow|Fast) Corner\):)\s*$',text)
                for i in range(1,len(blocks),2):
                    header=blocks[i];block=blocks[i+1].split('=====')[0]
                    vals=[]
                    for line in block.splitlines():
                        parts=line.split()
                        if len(parts)>=5:
                            try:wns=float(parts[-4]);tns=float(parts[-3]);fail=int(parts[-2]);total=int(parts[-1])
                            except ValueError:continue
                            vals.append((wns,fail,total))
                    if vals:timing.append({'class':header,'worstSlackNs':min(v[0] for v in vals),'failingEndpoints':sum(v[1] for v in vals),'reportedEndpoints':sum(v[2] for v in vals)})
                for line in text.split('Performance Summary:',1)[1].split('Design Summary',1)[0].splitlines():
                    m=re.match(r'\s*(\S+)\s+([\d.]+) MHz\s+([\d.]+) MHz\s+([\d.]+)\s+([\d.]+)\s+([-\d.]+)',line)
                    if m:performance.append({'clock':m[1],'requestedMHz':float(m[2]),'estimatedMHz':float(m[3]),'slackNs':float(m[6])})
                rec['allConstraintsMet']='Design Summary : All Constraints Met.' in text
                rec['status']='parsed' if performance else 'error'
            else:rec['reason']='不支持的格式或缺少资源/时序表'
        except OSError as e: rec['reason']=str(e)
        files.append(rec)
    ok=any(f['status']=='parsed' for f in files)
    text=f"LUT {resources['LUT']['used']:,} · DRM {resources.get('DRM_grid',{}).get('used','?')}" if 'LUT' in resources else f'{len(timing)}类时序汇总'
    return _row('resources','measured' if ok else 'condition_required','资源与时序',text if ok else '需要有效PDS报告',
        '导入实际PDS资源与时序表；不是当前下载位流身份或CDC覆盖认证。',
        {'files':files,'resources':resources,'timingSummaries':timing,'clocks':performance},[str(p) for p in paths],
        ['不同工程报告不得混用；汇总通过不替代完整CDC/例外审查。'],source='imported')
