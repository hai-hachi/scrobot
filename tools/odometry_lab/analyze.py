"""Offline metrics for SCROBOT Odometry Lab: use measurement-time interpolation.

Never compare the most recently received pose with the most recently received
truth pose: those samples can represent different instants in time.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

from core import wrap


def read_streams(path):
    streams=defaultdict(list)
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                t=float(row['stamp_s'])
                if not np.isfinite(t) or t<=0:
                    continue
                streams[row['stream']].append((t,float(row['x_m']),float(row['y_m']),
                    float(row['yaw_rad']),float(row['vx_mps']),float(row['wz_rps']),row['state']))
            except (ValueError, KeyError):
                continue
    output={}
    for name,rows in streams.items():
        # Keep last of duplicate stamps (distinct receive events are still logged).
        uniq={r[0]:r for r in rows}
        output[name]=sorted(uniq.values())
    return output


def nearest_states(records, times):
    ts=np.array([r[0] for r in records])
    idx=np.searchsorted(ts,times,side='right')-1
    return [records[max(0,i)][6] for i in idx]


def interpolate(records,t):
    if len(records)<2:
        return None
    a=np.asarray([r[:6] for r in records],dtype=float)
    if t[0]<a[0,0] or t[-1]>a[-1,0]:
        raise ValueError('Cannot extrapolate odometry beyond recorded interval')
    return np.stack([np.interp(t,a[:,0],np.unwrap(a[:,3]) if j==3 else a[:,j])
                     for j in range(1,6)],axis=1)


def relative_pose(x):
    p=x.copy()
    dx=p[:,0]-p[0,0]
    dy=p[:,1]-p[0,1]
    a=p[0,2]
    p[:,0]=dx*np.cos(a)+dy*np.sin(a)
    p[:,1]=-dx*np.sin(a)+dy*np.cos(a)
    p[:,2]=np.arctan2(np.sin(p[:,2]-a),np.cos(p[:,2]-a))
    return p


def rmse(x):
    return float(np.sqrt(np.mean(np.asarray(x)**2))) if len(x) else None


def calculate(truth, estimates, active_only=True):
    """Compare on matched simulation timestamps, after independent SE(2) origin alignment."""
    result={}
    for name,rows in estimates.items():
        if len(rows)<2 or len(truth)<2:
            continue
        t0=max(truth[0][0],rows[0][0]); t1=min(truth[-1][0],rows[-1][0])
        # Use truth stamps as evaluation timestamps, no nearest-message bias.
        times=np.asarray([r[0] for r in truth if t0<=r[0]<=t1])
        if len(times)<2:
            continue
        true=relative_pose(interpolate(truth,times))
        est=relative_pose(interpolate(rows,times))
        pos=np.hypot(true[:,0]-est[:,0],true[:,1]-est[:,1])
        ang=np.arctan2(np.sin(true[:,2]-est[:,2]),np.cos(true[:,2]-est[:,2]))
        truth_path=float(np.hypot(np.diff(true[:,0]),np.diff(true[:,1])).sum())
        active=np.array([s not in ('UNKNOWN','STATIC','START_DELAY','FINAL_SETTLE','DONE') and not s.startswith('STOP')
                         for s in nearest_states(truth,times)],dtype=bool)
        mask=active if active_only and active.any() else np.ones(len(times),dtype=bool)
        data={'matched_samples':int(mask.sum()),'matched_duration_s':float(times[-1]-times[0]),
            'ground_truth_path_m':truth_path,
            'position_rmse_m':rmse(pos[mask]),
            'heading_rmse_deg':math.degrees(rmse(ang[mask])),
            'max_position_error_m':float(pos[mask].max()),
            'endpoint_position_error_m':float(pos[-1]),
            'endpoint_heading_error_deg':abs(math.degrees(float(ang[-1]))),
            'drift_per_meter':float(pos[-1]/truth_path) if truth_path>1e-6 else None,
            'vx_rmse_mps':rmse((est[:,3]-(np.gradient(true[:,0],times)*np.cos(true[:,2])+np.gradient(true[:,1],times)*np.sin(true[:,2])))[mask]),
            'wz_rmse_rps':rmse((est[:,4]-np.gradient(np.unwrap(true[:,2]),times))[mask]),
            'scored_on':'active_motion' if active_only and active.any() else 'all_matched'}
        result[name]=data
    return result


def summarize_timing(samples_path):
    from core import StreamTiming
    streams={}
    with open(samples_path, newline='') as f:
        for row in csv.DictReader(f):
            key=row['stream']
            streams.setdefault(key,StreamTiming()).add(float(row['wall_time_s']),
                float(row['sim_recv_s']),float(row['stamp_s']))
    return {key:obj.summary() for key,obj in streams.items()}


def analyze(run_dir, plots=True):
    path=Path(run_dir)
    source=path/'samples.csv'
    if not source.exists():
        raise FileNotFoundError(source)
    streams=read_streams(source)
    truth=streams.get('ground_truth',[])
    if not truth:
        raise ValueError('No /evaluation/ground_truth_odom samples. Start Gazebo ground-truth bridge.')
    metrics={'accuracy':calculate(truth,{k:streams.get(k,[]) for k in
        ('wheel_odom','ekf_odom','encoder_odom')}),
        'timing':summarize_timing(source),
        'notes':['Trajectory comparisons use interpolated measurement timestamps and SE(2) start alignment.',
                 'Large interarrival gaps do not prove DDS packet loss.',
                 'Time age and RTF must be interpreted in simulation time vs wall time.']}
    live=path/'live.json'
    if live.exists():
        snapshot=json.loads(live.read_text())
        metrics['performance']={k:snapshot.get(k) for k in
            ('rtf','lab_cpu_percent','lab_memory_mib','encoder_updates','pending_samples')}
    (path/'summary.json').write_text(json.dumps(metrics,indent=2,allow_nan=False))
    with open(path/'summary.csv','w',newline='') as f:
        w=csv.writer(f); w.writerow(['stream','metric','value'])
        for category,value in metrics.items():
            if isinstance(value,dict):
                for stream,sub in value.items():
                    if isinstance(sub,dict):
                        for metric,x in sub.items():w.writerow([stream,metric,x])
                    else:w.writerow([category,stream,sub])
    if plots:
        try:
            import matplotlib
            matplotlib.use('Agg')
            from matplotlib import pyplot as plt
            fig,ax=plt.subplots(figsize=(7,6))
            for name in ('ground_truth','wheel_odom','ekf_odom','encoder_odom'):
                rows=streams.get(name,[])
                if len(rows)<2:continue
                a=relative_pose(np.asarray([r[1:6] for r in rows],dtype=float))
                ax.plot(a[:,0],a[:,1],label=name)
            ax.set(xlabel='Relative x (m)',ylabel='Relative y (m)',title='Odometry versus ground truth')
            ax.axis('equal');ax.grid(True,alpha=.25);ax.legend()
            fig.savefig(path/'trajectory.svg',bbox_inches='tight');plt.close(fig)
        except ImportError:
            pass
    return metrics


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run_dir')
    args=p.parse_args()
    print(json.dumps(analyze(args.run_dir),indent=2))
