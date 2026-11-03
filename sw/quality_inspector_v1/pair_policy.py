"""Observation-only Gamma interpretation and before/after evidence pictures."""
from pathlib import Path
import cv2
import numpy as np

STAGES=('baseline1','calibrated','baseline2')


def check_status(controller):
    """Keep configuration ownership and CNN guards; observe rather than force tone."""
    a5=controller.require_unchanged()
    caps=controller._fresh_capabilities()
    nn=caps.get('a7') or {}
    if nn.get('capabilityWord') not in (0x000f0003,0x000f0007):
        raise RuntimeError('板卡接口不匹配：需要R15或R16诊断标识，请确认下载版本。')
    if nn.get('enabled') is not False:
        raise RuntimeError('图像前后对比需要CNN关闭，以免测试期间自动改动参数；请先在控制台关闭CNN。')
    return {'a5':a5,'capabilities':caps}


def gamma_assessment(session):
    rows=[];problems=[]
    for name in STAGES:
        stage=session.get('stages',{}).get(name) or {};pair=[]
        for moment in ('beforeCapture','afterCapture'):
            snap=stage.get(moment) or {}
            gamma=((snap.get('capabilities') or {}).get('a8') or {}).get('decoded') or {}
            row={'stage':name,'moment':moment,**{k:gamma.get(k) for k in
                 ('valid','valid_count','frame_id','applied_bank','next_bank')}}
            rows.append(row);pair.append(row)
            if row['valid'] is not True or row['valid_count']!=2073600:
                problems.append('Gamma统计缺失或不是完整1080p帧')
            if type(row['applied_bank']) is not int or row['applied_bank'] not in range(5) or row['next_bank']!=row['applied_bank']:
                problems.append('存在混合档位、待切换或未知档位')
        a,b=pair
        if (type(a['frame_id']) is not int or type(b['frame_id']) is not int or
            not 0<((b['frame_id']-a['frame_id'])&0xffffffff)<0x80000000):
            problems.append('Gamma统计帧没有正常前进')
    banks={v['applied_bank'] for v in rows if type(v['applied_bank']) is int}
    if len(banks)!=1:problems.append('前后采集观测到不同Gamma档位')
    comparable=not problems
    bank=next(iter(banks)) if comparable else None
    if not comparable:
        detail='；'.join(dict.fromkeys(problems))+'；保留各组清晰度，暂不计算前后改善百分比。'
    elif bank==2:
        detail='三组采集边界均观测到恒等Gamma 2档；可比较输出域结果。'
    else:
        detail=f'三组采集边界均观测到Gamma {bank}档；仅作相同观测档位的输出域对比，非纯几何或线性光学测量。'
    override=session.get('gammaOverride') or {}
    fixed=(comparable and bank==2 and override.get('applied',{}).get('gamma_identity') is True
           and all((session['stages'][n].get('measurementBefore',{}).get('gamma_cmd_applied_seq')==override.get('applied',{}).get('gamma_cmd_applied_seq')
                    and session['stages'][n].get('measurementAfter',{}).get('gamma_cmd_applied_seq')==override.get('applied',{}).get('gamma_cmd_applied_seq')) for n in STAGES))
    if fixed:detail='R16已确认固定恒等Gamma，三组采集前后控制序号一致；允许几何输出域前后对比。'
    return {'outputComparisonAvailable':comparable,'pureGeometryComparisonAvailable':comparable and bank==2,
            'identityOverrideConfirmed':fixed,
            'observedBank':bank,'detail':detail,'snapshots':rows,
            'limits':['固定恒等控制已确认；原始包随报告保存。' if fixed else '采集前后快照不能证明每个HDMI像素均使用同一Gamma；没有锁定板端自动档位。',
                      '未校准OECF，恒等档位也不代表ISO认证或纯镜头MTF。']}


def apply_sfr_policy(summary,assessment):
    for item in summary:
        item['gammaComparisonAvailable']=assessment['outputComparisonAvailable']
        if not assessment['outputComparisonAvailable']:
            item['output_change_percent']=None
            item['normalized_change_percent']=None
            item['comparisonUnavailableReason']=assessment['detail']


def save_comparison(images,output):
    """Preserve native pixels; headers are outside the original image rectangles."""
    from PIL import Image,ImageDraw,ImageFont
    frames=[images[name][len(images[name])//2][1] for name in STAGES]
    if any(im.shape!=(1080,1920,3) for im in frames):
        raise ValueError('前后对照图需要三组原生1920×1080图像')
    canvas=np.zeros((1144,5760,3),dtype=np.uint8)
    for i,im in enumerate(frames):canvas[64:,i*1920:(i+1)*1920]=im
    path=Path(output)/'before_after_comparison.png'
    try:
        font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',32)
    except OSError:
        for i,text in enumerate(('BEFORE: zero distortion coefficients','AFTER: calibrated parameters, 0 degrees','REPEATED BEFORE: scene stability')):
            cv2.putText(canvas,text,(i*1920+24,43),cv2.FONT_HERSHEY_SIMPLEX,1.,(255,255,255),2)
        if not cv2.imwrite(str(path),canvas):raise OSError('无法保存校正前后对照图')
    else:
        picture=Image.fromarray(cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB));draw=ImageDraw.Draw(picture)
        for i,text in enumerate(('校正前：零畸变系数基线','校正后：真实标定参数0°','重复校正前：检查场景是否漂移')):
            draw.text((i*1920+24,10),text,font=font,fill='white')
        picture.save(path)
    return path
