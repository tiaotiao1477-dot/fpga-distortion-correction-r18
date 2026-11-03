"""Original-pixel SFR and matched checker-line measurements, no hardware."""
from pathlib import Path
import hashlib
import json
import csv
import math

import cv2
import numpy as np

from tools.sfr_measurement.sfr_measurement import analyze_edge, _fit_edge
from tools.sfr_measurement.checker_rois import (
    candidate_rois, candidate_rois_with_diagnostics, safe_edge_window, _canonicalize_grid,
)
from tools.sfr_measurement.analyze_pair import camera, inverse_points, normal_scale, PROFILE
from .pair_policy import gamma_assessment, apply_sfr_policy, save_comparison

TITLES={'geometry':'校正精度','sharpness':'清晰度','edges':'边缘与视野'}
LIMITS=['工程输出域测量，非ISO认证；未校准图卡精度/OECF。',
        'HDMI MJPEG解码后存PNG，仍包含压缩影响；PC未重采样图像。',
        '图卡覆盖范围不等于整个传感器视野；不设置虚构的官方合格线。']


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def row(key,status,value,detail,metrics=None,evidence=None,source='historical'):
    return dict(key=key,title=TITLES[key],status=status,value=value,detail=detail,
                metrics=metrics or {},evidence=evidence or [],limits=LIMITS.copy(),source=source)


def detect_grid(image):
    """LARGER discovers a visible rectangular grid; metadata supplies its size."""
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
    if hasattr(cv2,'findChessboardCornersSBWithMeta') and hasattr(cv2,'CALIB_CB_LARGER'):
        found,points,meta=cv2.findChessboardCornersSBWithMeta(gray,(5,4),flags=flags|cv2.CALIB_CB_LARGER)
        if found and meta is not None and points is not None:
            h,w=meta.shape[:2]
            if len(points.reshape(-1,2))==h*w and min(h,w)>=4:
                return _canonicalize_grid(points.reshape(h,w,2))
    else:
        found,points=cv2.findChessboardCornersSB(gray,(12,8),flags=flags)
        if found:return _canonicalize_grid(points.reshape(8,12,2))
    raise ValueError('未找到足够棋盘格角点；请让棋盘边缘清楚、避免反光并放稳。')


def tls(points):
    points=np.asarray(points,dtype=float)
    if len(points)<4:raise ValueError('每条格线至少需要4个匹配角点')
    center=points.mean(axis=0)
    _,_,vt=np.linalg.svd(points-center,full_matrices=False)
    distance=np.abs((points-center)@vt[-1])
    return dict(rms_px=float(np.sqrt(np.mean(distance**2))),max_px=float(distance.max()),
                points=points.tolist(),center=center.tolist(),normal=vt[-1].tolist())


def _read_inputs(path):
    session=json.loads(Path(path).read_text(encoding='utf-8'))
    stages=session.get('stages',{})
    if not all(k in stages for k in ('baseline1','calibrated','baseline2')):
        raise ValueError('需要校正前、校正后和重复基线三组完整采集')
    images={};hashes={}
    for name in ('baseline1','calibrated','baseline2'):
        spec=stages[name];record_path=Path(spec['captureRecord'])
        record=json.loads(record_path.read_text(encoding='utf-8'))
        expected=record.get('imageSha256',{})
        if not expected:raise ValueError('采集记录缺少原图SHA256')
        images[name]=[]
        for image_path in spec['images']:
            p=Path(image_path);digest=sha(p)
            if expected.get(p.name)!=digest:raise ValueError('原图SHA256不匹配：'+p.name)
            im=cv2.imread(str(p))
            if im is None or im.shape!=(1080,1920,3):raise ValueError('需要原生1920×1080采集图')
            images[name].append((str(p),im));hashes[str(p)]=digest
        if len(images[name])<3:raise ValueError('每组至少需要3张独立帧')
    return session,images,hashes


def _select_local(image,grid):
    if grid is not None:
        choices=[dict(v,endpoints=v['grid_endpoints'],length=v['edge_length_px'],
                      normal_spacing=v['normal_cell_spacing_px'])
                 for v in candidate_rois(grid,image.shape)]
    else:
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        lines=cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD).detect(gray)[0]
        choices=[]
        if lines is not None:
            for i,line in enumerate(lines.reshape(-1,4)):
                a,b=line[:2].astype(float),line[2:].astype(float);delta=b-a;length=float(np.linalg.norm(delta))
                angle=np.degrees(np.arctan2(min(abs(delta)),max(abs(delta))))
                if 75<=length<=180 and 1.5<=angle<=15:
                    choices.append(dict(edge_id=str(i),center=((a+b)/2).tolist(),endpoints=[a.tolist(),b.tolist()],
                        length=length,normal_spacing=length,
                        orientation='vertical' if abs(delta[1])>abs(delta[0]) else 'horizontal'))
    targets={'center':(.5,.5),'top_left':(.2,.2),'top_right':(.8,.2),'bottom_left':(.2,.8),'bottom_right':(.8,.8)}
    selected=[];used=set()
    for name,target in targets.items():
        for orientation in ('vertical','horizontal'):
            options=[v for v in choices if v['orientation']==orientation and v['edge_id'] not in used]
            if not options:continue
            chosen=min(options,key=lambda v:(np.linalg.norm(np.array(v['center'])/np.array([1920,1080])-target),v['edge_id']))
            used.add(chosen['edge_id']);selected.append(dict(chosen,name=name+'_'+orientation))
    return selected


def _sfr(images,grid,profile,output,gamma=None):
    k,d=camera(profile);selected=_select_local(images['baseline1'][len(images['baseline1'])//2][1],grid)
    if not selected:
        if grid is None:
            raise ValueError('未检测到足够棋盘格角点，且未发现长度75–180px、倾角1.5°–15°的有效斜边线。')
        _candidates, diagnostics=candidate_rois_with_diagnostics(
            grid,images['baseline1'][len(images['baseline1'])//2][1].shape)
        raise ValueError('已检测到棋盘格，但没有满足48×48原生ROI及单格安全边界的斜边窗口；'
                         f"候选{diagnostics['totalEdges']}条，可用{diagnostics['accepted']}条，"
                         f"尺寸/单格边界不足{diagnostics['tooSmallForCell']}条，"
                         f"图像越界{diagnostics['outsideImage']}条。请让棋盘在画面中占更大区域。")
    samples=[];artifacts=[]
    for mode,frames in images.items():
        for index,(path,image) in enumerate(frames):
            annotated=image.copy();gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY).astype(float)
            for spec in selected:
                points=np.array([spec['center'],*spec['endpoints']])
                if mode=='calibrated':points=inverse_points(points,k,d)
                center,a,b=points;normal_spacing=float(spec.get('normal_spacing',spec['length']))
                result=dict(name=spec['name'],stage=mode,frame=index+1,image=path,
                    orientation=spec['orientation'],roi=None,predictedCenter=center.tolist())
                window=None
                try:
                    if mode=='calibrated':
                        delta=b-a;delta/=np.linalg.norm(delta)
                        normal=np.array([-delta[1],delta[0]])
                        normal_spacing/=normal_scale(center,normal,profile)
                    window=safe_edge_window(a,b,normal_spacing,image.shape)
                    x,y,w,h=(window[key] for key in ('x','y','width','height'))
                    result['roi']=[int(x),int(y),int(w),int(h)]
                    crop=gray[y:y+h,x:x+w];r=analyze_edge(crop,orientation=spec['orientation'])
                    fit=crop if spec['orientation']=='vertical' else crop.T
                    slope,intercept,*_=_fit_edge(fit);along=(fit.shape[0]-1)/2;cross=slope*along+intercept
                    fitted=np.array([x+cross,y+along] if spec['orientation']=='vertical' else [x+along,y+cross])
                    normal=np.array([1,-slope] if spec['orientation']=='vertical' else [-slope,1]);normal/=np.linalg.norm(normal)
                    error=float(abs((fitted-center)@normal))
                    if error>6:raise ValueError('同一物理边匹配偏差超过6px')
                    scale=normal_scale(fitted,normal,profile) if mode=='calibrated' else 1.
                    result.update(r,edgeMatchErrorPx=error,matchedCenter=fitted.tolist(),scale=scale,
                        mtf50_source_cpp=None if r['mtf50_cpp'] is None else r['mtf50_cpp']/scale,
                        low=float(np.percentile(crop,10)),high=float(np.percentile(crop,90)))
                    color=(0,220,0)
                except (ValueError,AssertionError) as exc:
                    result.update(error=str(exc),qc={'passed':False});color=(0,0,255)
                samples.append(result)
                if window is not None:
                    cv2.rectangle(annotated,(x,y),(x+w-1,y+h-1),color,2)
                    cv2.putText(annotated,spec['name'],(x,max(y-4,18)),cv2.FONT_HERSHEY_SIMPLEX,.42,color,1)
            if index==len(frames)//2:
                p=output/(mode+'_rois.png');cv2.imwrite(str(p),annotated);artifacts.append(str(p))
    summary=[]
    for spec in selected:
        item={'name':spec['name'],'orientation':spec['orientation']}
        for mode in images:
            valid=[v for v in samples if v['name']==spec['name'] and v['stage']==mode and v['qc']['passed']]
            metrics={'accepted':len(valid),'expected':len(images[mode])}
            for key in ('mtf50_cpp','mtf10_cpp','mtf50_source_cpp','scale','low','high'):
                values=[v[key] for v in valid if v.get(key) is not None]
                metrics[key]=dict(median=float(np.median(values)),min=float(min(values)),max=float(max(values))) if values else None
            if valid:metrics['center']=np.mean([v['matchedCenter'] for v in valid],axis=0).tolist()
            item[mode]=metrics
        b1,b2,cal=[item[m] for m in ('baseline1','baseline2','calibrated')]
        complete=all(item[m]['accepted']==item[m]['expected'] for m in images)
        item['stable']=False
        if complete:
            item['drift_px']=float(np.linalg.norm(np.array(b1['center'])-b2['center']))
            item['plateau_difference']=max(abs(b1[t]['median']-b2[t]['median']) for t in ('low','high'))
            item['stable']=item['drift_px']<=1 and item['plateau_difference']<=8
            if all(v['mtf50_cpp'] is not None for v in (b1,b2,cal)):
                base=(b1['mtf50_cpp']['median']+b2['mtf50_cpp']['median'])/2
                item['output_change_percent']=100*(cal['mtf50_cpp']['median']/base-1)
                item['normalized_change_percent']=100*(cal['mtf50_source_cpp']['median']/base-1)
        summary.append(item)
    values=[v['mtf50_cpp'] for v in samples if v['stage']=='calibrated' and v['qc']['passed'] and v['mtf50_cpp'] is not None]
    if gamma is not None:apply_sfr_policy(summary,gamma)
    save(output/'sfr_results.json',dict(selected=selected,samples=samples,summary=summary,profile=profile,gammaAssessment=gamma,
         input_resampled=False,oecf_calibrated=False,scope='engineering output-domain SFR, not ISO certified'))
    with (output/'sfr_curves.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(['roi','stage','frame','cycles/outputpixel','SFR'])
        for r in samples:
            if r['qc']['passed']:writer.writerows((r['name'],r['stage'],r['frame'],f,v) for f,v in zip(r['freq_cpp'],r['sfr']))
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    fig,axes=plt.subplots(math.ceil(len(selected)/2),2,figsize=(11,max(4,2.5*math.ceil(len(selected)/2))),squeeze=False,layout='constrained')
    for ax,spec in zip(axes.flat,selected):
        for mode,color in zip(images,('tab:blue','tab:orange','tab:green')):
            group=[v for v in samples if v['name']==spec['name'] and v['stage']==mode and v['qc']['passed']]
            if group:
                curves=np.array([v['sfr'] for v in group]);freq=group[0]['freq_cpp']
                ax.plot(freq,np.median(curves,axis=0),color=color,label=mode)
                ax.fill_between(freq,curves.min(axis=0),curves.max(axis=0),color=color,alpha=.12)
        ax.axhline(.5,ls='--',color='gray',lw=.7)
        ax.set(xlim=(0,.5),ylim=(0,1.15),title=spec['name'],xlabel='cycles/output pixel',ylabel='SFR')
        ax.grid(alpha=.2);ax.legend(fontsize=7)
    for ax in list(axes.flat)[len(selected):]:ax.set_visible(False)
    fig.suptitle('Engineering output-domain SFR: median/range, not ISO certified')
    p=output/'sfr_curves.png';fig.savefig(p,dpi=130);plt.close(fig);artifacts.append(str(p))
    return samples,summary,values,artifacts


def _geometry(images,reference,profile,output):
    k,d=camera(profile);ref=reference.reshape(-1,2);shape=reference.shape[:2]
    frames=[]
    for mode,group in images.items():
        for idx,(path,image) in enumerate(group):
            grid=detect_grid(image);actual=grid.reshape(-1,2)
            predicted=inverse_points(ref,k,d) if mode=='calibrated' else ref
            dist=np.linalg.norm(predicted[:,None,:]-actual[None,:,:],axis=2)
            indices=np.argmin(dist,axis=1);errors=dist[np.arange(len(ref)),indices]
            valid=errors<5
            if len(set(indices[valid]))!=int(valid.sum()):raise ValueError('角点对应存在歧义，请保持棋盘不动')
            matched=np.full_like(ref,np.nan);matched[valid]=actual[indices[valid]]
            frames.append(dict(stage=mode,frame=idx+1,image=path,matched=matched.reshape(*shape,2),valid=valid))
    common=np.logical_and.reduce([v['valid'] for v in frames]).reshape(shape)
    line_specs=[]
    for r in range(shape[0]):
        if common[r].sum()>=5:line_specs.append(('row'+str(r),r,None,common[r]))
    for c in range(shape[1]):
        if common[:,c].sum()>=4:line_specs.append(('col'+str(c),None,c,common[:,c]))
    if len(line_specs)<4:raise ValueError('匹配格线覆盖不足；请将更多棋盘放入画面且保持不动')
    summaries={};perframe=[];artifacts=[]
    for mode,group in images.items():
        for item in [v for v in frames if v['stage']==mode]:
            fits=[]
            for label,r,c,mask in line_specs:
                points=item['matched'][r,mask] if r is not None else item['matched'][mask,c]
                fits.append(dict(name=label,**tls(points)))
            perframe.append(dict(stage=mode,frame=item['frame'],lineCount=len(fits),lines=fits,
                meanRmsPx=float(np.mean([v['rms_px'] for v in fits])),maximumDeviationPx=max(v['max_px'] for v in fits)))
            if item['frame']==len(group)//2+1:
                im=cv2.imread(item['image'])
                for fit in fits:
                    points=np.array(fit['points']);p=tuple(np.round(points[0]).astype(int));q=tuple(np.round(points[-1]).astype(int))
                    cv2.line(im,p,q,(0,230,100),1)
                    for pt in points:cv2.circle(im,tuple(np.round(pt).astype(int)),3,(0,120,255),1)
                p=output/(mode+'_straightness.png');cv2.imwrite(str(p),im);artifacts.append(str(p))
        subset=[v for v in perframe if v['stage']==mode]
        summaries[mode]={'meanLineRmsMedianPx':float(np.median([v['meanRmsPx'] for v in subset])),
                         'maximumDeviationPx':max(v['maximumDeviationPx'] for v in subset),'lines':len(line_specs)}
    b1,b2,cal=[summaries[m]['meanLineRmsMedianPx'] for m in ('baseline1','baseline2','calibrated')]
    valid_frames=[v['matched'].reshape(-1,2) for v in frames]
    b1m=np.nanmean([v['matched'] for v in frames if v['stage']=='baseline1'],axis=0)
    b2m=np.nanmean([v['matched'] for v in frames if v['stage']=='baseline2'],axis=0)
    drift=float(np.nanmax(np.linalg.norm(b1m[common]-b2m[common],axis=-1)))
    result=dict(stages=summaries,perFrame=perframe,commonCorners=int(common.sum()),gridShape=list(shape),
                repeatMaximumCornerDriftPx=drift,reductionPercent=100*(1-cal/((b1+b2)/2)) if b1+b2>0 else None,
                scope='matched physical checker lines perpendicular TLS distances in output pixels')
    save(output/'geometry_results.json',result);artifacts.append(str(output/'geometry_results.json'))
    status='measured' if drift<=1 else 'condition_required'
    return row('geometry',status,f'{(b1+b2)/2:.3f} → {cal:.3f} px RMS',
        f'{len(line_specs)}条共同格线 / {int(common.sum())}角点；重复基线最大漂移{drift:.3f}px；'+
        ('已取得匹配比较' if drift<=1 else '图卡移动，前后比较无效'),result,artifacts)


def analyze_session(session_path,output,selected):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False);results={}
    try:session,images,hashes=_read_inputs(session_path)
    except (OSError,ValueError,KeyError) as exc:
        for key in selected:results[key]=row(key,'error','证据无效',str(exc))
        save(output/'results.json',results);return results
    profile_path=Path(session.get('profilePath') or PROFILE)
    profile=json.loads(profile_path.read_text(encoding='utf-8'))
    gamma=gamma_assessment(session)
    comparison=save_comparison(images,output)
    comparison_evidence=[str(comparison)]+[images[m][len(images[m])//2][0] for m in ('baseline1','calibrated','baseline2')]
    save(output/'gamma_assessment.json',gamma)
    image=images['baseline1'][len(images['baseline1'])//2][1]
    try:grid=detect_grid(image)
    except ValueError:grid=None
    if 'geometry' in selected:
        try:
            if grid is None:raise ValueError('未找到共同棋盘格；请减少反光并让格线清楚')
            results['geometry']=_geometry(images,grid,profile,output)
        except ValueError as exc:results['geometry']=row('geometry','condition_required','需要清楚棋盘格',str(exc))
    if 'sharpness' in selected:
        try:
            samples,summary,values,artifacts=_sfr(images,grid,profile,output,gamma)
            good=[v for v in summary if v['stable']]
            status='measured' if good and values else 'condition_required'
            artifacts += [str(output/'sfr_results.json'),str(output/'sfr_curves.csv')]
            centers={v['name']:v['calibrated'].get('mtf50_cpp') for v in summary if v['name'].startswith('center_')}
            center_value=(f"中心 {centers['center_vertical']['median']:.3f}/{centers['center_horizontal']['median']:.3f} cy/px"
                if centers.get('center_vertical') and centers.get('center_horizontal') else
                f'{min(values):.3f}–{max(values):.3f} cy/px' if values else '斜边质量检查未通过')
            rejection_counts={}
            for sample in samples:
                reason=sample.get('error')
                if reason:rejection_counts[reason]=rejection_counts.get(reason,0)+1
            results['sharpness']=row('sharpness',status,center_value,
                f'{len(good)}/{len(summary)}处斜边完整且基线稳定；'+gamma['detail'],
                {'rois':summary,'accepted':sum(v.get('qc',{}).get('passed') is True for v in samples),
                 'total':len(samples),'qcRejections':rejection_counts,'gammaAssessment':gamma},artifacts)
            if not gamma['outputComparisonAvailable'] and values:
                results['sharpness']['value']='仅当前输出：'+center_value
        except (ValueError,TypeError) as exc:
            message=str(exc)
            label=('未检测到棋盘格/斜边' if '未检测到' in message else
                   '无满足安全尺寸的斜边ROI' if '48×48原生ROI' in message else
                   '斜边质量检查未通过' if '斜边' in message else '清晰度条件未满足')
            results['sharpness']=row('sharpness','condition_required',label,message)
    if 'edges' in selected:
        audit=Path(session['stages']['calibrated']['configDir'])/'crop_report.json'
        if audit.is_file():
            details=json.loads(audit.read_text(encoding='utf-8'))
            invalid=details.get('invalidFetch4Count');total=details.get('count')
            label=(f'0°无效映射 {invalid}/{total}' if invalid is not None and total else '0°映射范围已记录')
            results['edges']=row('edges','measured',label,'基于实际配置的全光栅四点取样审计；不是按黑色像素判定黑边',
                {'configurationRangeAudit':details,'physicalEdgeBreakageVerified':False},[str(audit)])
            results['edges']['limits']+=['只记录本次0°模型/配置覆盖，未自动证明所有角度无拉伸或断裂。']
        else:results['edges']=row('edges','condition_required','缺少映射配置','先执行一次真实参数校正对比以取得配置审计。')
    for key,value in results.items():
        value['metrics'].setdefault('gammaAssessment',gamma)
        value['evidence'] += comparison_evidence+[str(output/'gamma_assessment.json'),str(Path(session_path).resolve()),str(output/'inputs.json')]
        value['limits']+=gamma['limits']
        if key=='geometry':value['detail']+='；'+gamma['detail']
    save(output/'inputs.json',{'session':str(session_path),'sessionSha256':sha(session_path),
        'images':hashes,'profile':str(profile_path),'profileSha256':sha(profile_path),
        'analysisToolSha256':sha(__file__),
        'gammaPolicyToolSha256':sha(Path(__file__).parent/'pair_policy.py')})
    save(output/'results.json',results)
    return results
