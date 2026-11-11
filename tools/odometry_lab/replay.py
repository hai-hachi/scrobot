"""Real-vs-simulation wheel-RPM comparison with explicit source columns.

This tests motor response fidelity; RPM is not measured robot pose ground truth.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def read_real(path,time_col,rpm_col):
    result=[]
    with open(path,newline='') as f:
        reader=csv.DictReader(f)
        for row in reader:
            try:
                t=float(row[time_col]);rpm=float(row[rpm_col])
                if math.isfinite(t) and math.isfinite(rpm):result.append((t,rpm))
            except (KeyError,ValueError,TypeError):continue
    result.sort()
    if len(result)<2:raise ValueError(f'Need at least 2 valid rows in {path}; check CSV column names')
    return result


def sim_records(path):
    samples={'left':[],'right':[]}
    with open(path,newline='') as f:
        for row in csv.DictReader(f):
            try:
                t=float(row['stamp_s'])
                samples['left'].append((t,float(row['left_rpm'])))
                samples['right'].append((t,float(row['right_rpm'])))
            except (KeyError,ValueError):continue
    return samples


def summarize(rows):
    r=[v for _,v in rows];dt=[b[0]-a[0] for a,b in zip(rows,rows[1:]) if b[0]>a[0]]
    return {'samples':len(r),'duration_s':rows[-1][0]-rows[0][0],
        'mean_rpm':statistics.mean(r),'rpm_std':statistics.pstdev(r),
        'rmse_vs_90rpm':math.sqrt(statistics.mean((v-90)**2 for v in r)),
        'mean_sample_hz':1./statistics.mean(dt) if dt else None}


def compare_time_aligned(real,sim):
    """Align first timestamp; use relative elapsed time, no arbitrary cross-correlation."""
    a=[(t-real[0][0],v) for t,v in real]
    b=[(t-sim[0][0],v) for t,v in sim]
    if len(b)<2:return {'comparison':'insufficient_sim_samples'}
    i=0;errors=[]
    for t,v in a:
        if not b[0][0]<=t<=b[-1][0]:continue
        while i+1<len(b)-1 and b[i+1][0]<t:i+=1
        t0,v0=b[i];t1,v1=b[i+1]
        pred=v0+(v1-v0)*(t-t0)/(t1-t0) if t1>t0 else v1
        errors.append(pred-v)
    return {'comparison':'relative_start_timestamp_alignment',
            'matched_samples':len(errors),
            'sim_vs_real_rpm_rmse':math.sqrt(statistics.mean(e*e for e in errors)) if errors else None,
            'warning':'Different command start times or acceleration profiles distort this RMSE'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--left',required=True);p.add_argument('--right',required=True)
    p.add_argument('--time-column',required=True);p.add_argument('--rpm-column',required=True)
    p.add_argument('--sim-wheel-speeds',help='wheel_speeds.csv from an odometry lab run')
    p.add_argument('--output',default='hardware_replay_summary.json')
    a=p.parse_args()
    real={side:read_real(getattr(a,side),a.time_column,a.rpm_column) for side in ('left','right')}
    result={side:{'real':summarize(values)} for side,values in real.items()}
    if a.sim_wheel_speeds:
        sim=sim_records(a.sim_wheel_speeds)
        for side in ('left','right'):
            if len(sim[side])>1:
                result[side]['sim']=summarize(sim[side])
                result[side]['comparison']=compare_time_aligned(real[side],sim[side])
    Path(a.output).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
