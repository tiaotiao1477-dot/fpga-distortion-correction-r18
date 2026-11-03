"""Analyze captured physical checker edges without requiring a complete grid.

Only coordinate predictions use the calibration model. Image pixels are never
warped, averaged, denoised or resampled for measurement.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.sfr_measurement.sfr_measurement import analyze_edge, _fit_edge

PROFILE = ROOT / 'data/calibration/ov5640_user_photos_20261022/board_validation_20261023/measured_native_profile/camera_profile_1080_runtime.json'
TARGETS = {'center': (960, 540), 'chart_top_left': (580, 180),
           'chart_top_right': (1500, 180), 'chart_bottom_left': (580, 720),
           'chart_bottom_right': (1500, 720)}
STAGES = ('baseline1', 'calibrated', 'baseline2')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def candidates(gray):
    lines = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD).detect(gray)[0]
    result = []
    for i, line in enumerate(lines.reshape(-1, 4)):
        a, b = line[:2].astype(float), line[2:].astype(float)
        delta = b-a
        length = float(np.linalg.norm(delta))
        center = (a+b)/2
        orientation = 'vertical' if abs(delta[1]) > abs(delta[0]) else 'horizontal'
        slant = float(np.degrees(np.arctan2(min(abs(delta)), max(abs(delta)))))
        # Visible printed chart interior, established from the saved overview.
        if not (75 <= length <= 120 and 1.5 <= slant <= 15 and
                500 < center[0] < 1620 and 80 < center[1] < 790):
            continue
        result.append(dict(id=i, center=center.tolist(), endpoints=[a.tolist(), b.tolist()],
                           length=length, orientation=orientation, slant=slant))
    return result


def select(items):
    """Pure geometry selection; never considers MTF or SFR quality/results."""
    used, selected = set(), []
    for region, target in TARGETS.items():
        for orientation in ('vertical', 'horizontal'):
            choices = [v for v in items if v['orientation'] == orientation and v['id'] not in used]
            item = min(choices, key=lambda v: (np.linalg.norm(np.array(v['center'])-target), v['id']))
            used.add(item['id'])
            selected.append(dict(item, name=region+'_'+orientation, region=region, target=list(target)))
    return selected


def camera(profile):
    k = np.array([[profile['fx'], 0, profile['cx']], [0, profile['fy'], profile['cy']], [0, 0, 1.]])
    d = np.array([profile[key] for key in ('k1', 'k2', 'p1', 'p2')])
    return k, d


def inverse_points(points, k, d):
    profile=dict(fx=k[0,0],fy=k[1,1],cx=k[0,2],cy=k[1,2],
                 k1=d[0],k2=d[1],p1=d[2],p2=d[3])
    result=[]
    for source in np.array(points,dtype=float).reshape(-1,2):
        guess=source.copy()
        for _ in range(20):
            residual=forward(guess,profile)-source
            if np.linalg.norm(residual)<1e-9:
                break
            eps=.01
            jac=np.column_stack([(forward(guess+np.eye(2)[i]*eps,profile)-
                                  forward(guess-np.eye(2)[i]*eps,profile))/(2*eps) for i in range(2)])
            guess-=np.linalg.solve(jac,residual)
        if np.linalg.norm(forward(guess,profile)-source)>1e-7:
            raise ValueError('inverse Brown coordinate solve did not converge')
        result.append(guess)
    return np.array(result)


def forward(point, profile):
    x, y = (point-np.array([profile['cx'], profile['cy']]))/np.array([profile['fx'], profile['fy']])
    r2 = x*x+y*y
    radial = 1+profile['k1']*r2+profile['k2']*r2*r2
    xx = x*radial+2*profile['p1']*x*y+profile['p2']*(r2+2*x*x)
    yy = y*radial+profile['p1']*(r2+2*y*y)+2*profile['p2']*x*y
    return np.array([xx*profile['fx']+profile['cx'], yy*profile['fy']+profile['cy']])


def normal_scale(point, normal, profile):
    eps = 0.01
    jac = np.column_stack([(forward(point+np.eye(2)[i]*eps, profile)-
                            forward(point-np.eye(2)[i]*eps, profile))/(2*eps) for i in range(2)])
    return float(1/np.linalg.norm(np.linalg.solve(jac.T, normal)))


def bounds(item, calibrated, k, d):
    points = np.array([item['center'], *item['endpoints']])
    if calibrated:
        points = inverse_points(points, k, d)
    center, a, b = points
    delta = b-a
    orientation = item['orientation']
    length = min(68, int(np.linalg.norm(delta)*0.65))
    width, height = (52, length) if orientation == 'vertical' else (length, 52)
    x, y = np.round(center-np.array([width, height])/2).astype(int)
    return dict(x=int(x), y=int(y), width=width, height=height,
                predictedCenter=center.tolist(), predictedEndpoints=[a.tolist(), b.tolist()])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    session = json.loads(args.session.read_text(encoding='utf-8'))
    assert session['restoredMeasured0'] is True
    assert args.output.exists() is False
    args.output.mkdir(parents=True)
    profile = json.loads(PROFILE.read_text(encoding='utf-8'))
    k, d = camera(profile)
    overview = cv2.imread(session['stages']['baseline1']['images'][2])
    items = candidates(cv2.cvtColor(overview, cv2.COLOR_BGR2GRAY))
    selected = select(items)
    write(args.output/'selection.json', dict(rule='nearest target per orientation, geometry only; no quality-based substitution',
          chartInteriorBounds=[500, 80, 1620, 790], targets=TARGETS, candidates=items, selected=selected))
    results, hashes = [], {}
    for stage in STAGES:
        for frame_index, image_path in enumerate(session['stages'][stage]['images']):
            hashes[image_path] = sha(image_path)
            image = cv2.imread(image_path)
            assert image.shape == (1080, 1920, 3)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(float)
            annotated = image.copy()
            for item in selected:
                spec = bounds(item, stage == 'calibrated', k, d)
                x,y,w,h = [spec[key] for key in ('x','y','width','height')]
                row = dict(stage=stage, frame=frame_index+1, image=image_path, name=item['name'],
                           region=item['region'], orientation=item['orientation'], roi=spec)
                try:
                    assert 0<=x and 0<=y and x+w<=1920 and y+h<=1080
                    crop = gray[y:y+h, x:x+w]
                    result = analyze_edge(crop, orientation=item['orientation'])
                    fit_image = crop if item['orientation']=='vertical' else crop.T
                    slope, intercept, *_ = _fit_edge(fit_image)
                    along = (fit_image.shape[0]-1)/2
                    cross = slope*along+intercept
                    center = np.array([x+cross,y+along] if item['orientation']=='vertical' else [x+along,y+cross])
                    normal = np.array([1,-slope] if item['orientation']=='vertical' else [-slope,1])
                    normal /= np.linalg.norm(normal)
                    error = float(abs(np.dot(center-np.array(spec['predictedCenter']), normal)))
                    if error > 6:
                        raise ValueError(f'edge matching error {error:.3f}px exceeds 6px')
                    scale = normal_scale(center, normal, profile) if stage=='calibrated' else 1.0
                    row.update(result, matchedCenter=center.tolist(), edgeMatchErrorPx=error,
                               sourceNormalPixelsPerOutputPixel=scale,
                               mtf50_source_cpp=None if result['mtf50_cpp'] is None else result['mtf50_cpp']/scale,
                               mean=float(crop.mean()), low=float(np.percentile(crop,10)), high=float(np.percentile(crop,90)))
                    color=(0,220,0)
                except (ValueError, AssertionError) as exc:
                    row.update(error=str(exc), qc={'passed':False})
                    color=(0,0,255)
                results.append(row)
                cv2.rectangle(annotated,(x,y),(x+w-1,y+h-1),color,2)
                cv2.putText(annotated,item['name'],(x,max(20,y-5)),cv2.FONT_HERSHEY_SIMPLEX,.43,color,1,cv2.LINE_AA)
            if frame_index==2:
                cv2.imwrite(str(args.output/(stage+'_rois.png')),annotated)
    summary=[]
    for item in selected:
        row=dict(name=item['name'],orientation=item['orientation'],region=item['region'])
        for stage in STAGES:
            subset=[v for v in results if v['name']==item['name'] and v['stage']==stage]
            valid=[v for v in subset if v['qc']['passed']]
            stats={'acceptedFrames':len(valid),'rejected':[v.get('error') for v in subset if not v['qc']['passed']]}
            for metric in ('mtf50_cpp','mtf10_cpp','mtf50_source_cpp','angle_deg','fit_residual','edgeMatchErrorPx',
                           'sourceNormalPixelsPerOutputPixel','mean','low','high'):
                vals=[v[metric] for v in valid if v.get(metric) is not None]
                stats[metric]=dict(median=float(np.median(vals)),min=float(min(vals)),max=float(max(vals))) if vals else None
            if valid:
                stats['meanMatchedCenter']=np.mean([v['matchedCenter'] for v in valid],axis=0).tolist()
            row[stage]=stats
        if all(row[s]['acceptedFrames']==5 for s in STAGES):
            b1,b2,cal=[row[s] for s in ('baseline1','baseline2','calibrated')]
            displacement=np.array(b2['meanMatchedCenter'])-np.array(b1['meanMatchedCenter'])
            row['baselineRepeatDisplacementPx']=float(np.linalg.norm(displacement))
            row['baselineRepeatLevelDifference']=max(abs(b2[t]['median']-b1[t]['median']) for t in ('low','high'))
            row['pairStable']=row['baselineRepeatDisplacementPx']<=1 and row['baselineRepeatLevelDifference']<=8
            row['mtf50BaselineRepeatPercent']=100*(b2['mtf50_cpp']['median']/b1['mtf50_cpp']['median']-1)
            baseline=(b1['mtf50_cpp']['median']+b2['mtf50_cpp']['median'])/2
            row['mtf50OutputChangePercent']=100*(cal['mtf50_cpp']['median']/baseline-1)
            row['mtf50ScaleNormalizedChangePercent']=100*(cal['mtf50_source_cpp']['median']/baseline-1)
        else:
            row['pairStable']=False
        summary.append(row)
    write(args.output/'results.json',dict(scope='engineering output-domain slanted-edge SFR; not ISO certified',
          noPixelResampling=True, isoCertificationClaim=False, oecfCalibrated=False,
          baseline='same FPGA sampler, zero Brown coefficients, not sensor RAW',
          repeatGates=dict(maximumEdgeDisplacementPx=1,maximumPlateauLevelDifference=8),
          coordinateMapping='Newton inverse Brown coordinate prediction with residual <=1e-7 pixel; original rectangular pixel crops',
          scaleNormalization='source normal pixels/output pixel = 1/||J^-T n_output||; float model derived',
          sessionSha256=sha(args.session), profileSha256=sha(PROFILE), toolSha256=sha(__file__),
          analyzerSha256=sha(ROOT/'tools/sfr_measurement/sfr_measurement.py'),imageHashes=hashes,
          summary=summary, results=results))
    with (args.output/'curves.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(['region_orientation','stage','frame','cycles_per_output_pixel','SFR'])
        for row in results:
            if row['qc']['passed']:
                writer.writerows((row['name'],row['stage'],row['frame'],f,v) for f,v in zip(row['freq_cpp'],row['sfr']))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(5,2,figsize=(12,18),layout='constrained')
    for ax,item in zip(axes.flat,selected):
        for stage,color in zip(STAGES,('tab:blue','tab:orange','tab:green')):
            subset=[v for v in results if v['name']==item['name'] and v['stage']==stage and v['qc']['passed']]
            if subset:
                curves=np.array([v['sfr'] for v in subset]);f=subset[0]['freq_cpp']
                ax.plot(f,np.median(curves,axis=0),label=stage,color=color)
                ax.fill_between(f,curves.min(axis=0),curves.max(axis=0),alpha=.12,color=color)
        ax.axhline(.5,ls='--',lw=.7,color='gray');ax.set(xlim=(0,.5),ylim=(0,1.15),title=item['name'],xlabel='cycles/output pixel',ylabel='SFR')
        ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Engineering e-SFR: per-frame median and range; not ISO certified')
    fig.savefig(args.output/'curves.png',dpi=140);plt.close(fig)
    for row in summary:
        print(row['name'],[row[s]['acceptedFrames'] for s in STAGES],
              'stable',row['pairStable'],'change%',round(row.get('mtf50OutputChangePercent',float('nan')),2))


if __name__=='__main__':
    main()
