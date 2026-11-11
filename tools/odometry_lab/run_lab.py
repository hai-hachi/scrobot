#!/usr/bin/env python3
"""Start Odometry Lab beside an already-running SCROBOT simulation/control stack."""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

HERE=Path(__file__).resolve().parent
DEFAULT_CONFIG=HERE/'config'/'lab.yaml'


def config_from(path):
    config=yaml.safe_load(Path(path).read_text())
    if not isinstance(config,dict):raise ValueError('Expected a YAML configuration mapping')
    return config


def unique_run_dir(config,name=None):
    root=Path(config['run_root']).expanduser()
    root.mkdir(parents=True,exist_ok=True)
    base=name or datetime.now().strftime('odom_%Y%m%d_%H%M%S')
    path=root/base
    count=1
    while path.exists():
        path=root/f'{base}_{count:02d}';count+=1
    path.mkdir()
    return path


def test_command(cfg,scenario):
    m=cfg['motion']
    params={
        'use_sim_time': 'true', 'test_type':scenario,
        'publish_rate':m['command_publish_hz'],
        'linear_speed':m['linear_speed_mps'],
        'straight_distance':m['straight_distance_m'],
        'angular_speed':m['angular_speed_rps'],
        'arc_linear_speed':m['arc_linear_speed_mps'],
        'arc_angular_speed':m['arc_angular_speed_rps'],
        'start_delay':m['start_delay_s'],
        'stop_duration':m['stop_duration_s'],
        'static_duration':m['static_duration_s'],
        'final_settle_duration':m['final_settle_duration_s'],
    }
    cmd=['ros2','run','scrobot_evaluation','local_odom_test_runner','--ros-args']
    for key,val in params.items():cmd+=['-p',f'{key}:={val}']
    return cmd


def stop_motion():
    """Best-effort simulated safety stop if a test is interrupted.

    The existing test runner normally restores AUTO after a completed test.
    Operators must still use their hardware E-stop on a real robot.
    """
    zero='{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'
    for cmd in (
        ['ros2','topic','pub','--once','/cmd_vel_manual_input',
         'geometry_msgs/msg/TwistStamped',zero],
        ['ros2','service','call','/control/set_manual_mode',
         'std_srvs/srv/SetBool','{data: false}']):
        try:
            subprocess.run(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                           timeout=3,check=False)
        except (OSError,subprocess.TimeoutExpired):
            pass


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',default=str(DEFAULT_CONFIG))
    ap.add_argument('--scenario',choices=['none','static','straight','rotate','arc','suite'],default='none')
    ap.add_argument('--run-name',default=None)
    ap.add_argument('--publish-hz',type=float,default=None,
        help='Lab-only encoder /odometry_lab/encoder_odom publish Hz; does not change production /odom')
    ap.add_argument('--sample-hz',type=float,default=None)
    ap.add_argument('--estimator-hz',type=float,default=None)
    ap.add_argument('--analyze-only',metavar='DIRECTORY',default=None)
    args=ap.parse_args(argv)
    if args.analyze_only:
        from analyze import analyze
        print(json.dumps(analyze(args.analyze_only),indent=2))
        return 0
    cfg=config_from(args.config)
    for flag,key in [('publish_hz','publish_hz'),('sample_hz','encoder_sample_hz'),('estimator_hz','estimator_hz')]:
        value=getattr(args,flag)
        if value is not None:cfg['rates'][key]=value
    for key in ('publish_hz','encoder_sample_hz','estimator_hz'):
        if not 1.<=float(cfg['rates'][key])<=500.:
            raise ValueError(f'{key} must be between 1 and 500 Hz')
    scenario=args.scenario
    run_dir=unique_run_dir(cfg,args.run_name)
    (run_dir/'config_snapshot.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    (run_dir/'metadata.json').write_text(json.dumps({'scenario':scenario,
        'start_wall_iso':datetime.now().isoformat(),
        'production_odom_rate_unchanged':True},indent=2))
    env=os.environ.copy()
    monitor=None; runner=None; load=None
    stop_requested=False
    def handle_signal(_sig,_frame):
        nonlocal stop_requested
        stop_requested=True
    signal.signal(signal.SIGTERM,handle_signal)
    signal.signal(signal.SIGINT,handle_signal)
    try:
        stress=float(cfg.get('stress',{}).get('cpu_duty',0.0))
        if not 0.<=stress<=.75:raise ValueError('Stress duty must be 0..0.75')
        if stress>0:
            with open(run_dir/'stress.log','w') as f:
                load=subprocess.Popen([sys.executable,str(HERE/'stress.py'),str(stress)],
                                      stdout=f,stderr=subprocess.STDOUT,env=env)
        with open(run_dir/'monitor.log','w') as ml:
            monitor=subprocess.Popen([sys.executable,str(HERE/'monitor_entry.py'),
                str(args.config),str(run_dir),str(run_dir/'config_snapshot.yaml')],
                stdout=ml,stderr=subprocess.STDOUT,env=env)
            if scenario!='none':
                with open(run_dir/'motion.log','w') as tl:
                    runner=subprocess.Popen(test_command(cfg,scenario),
                        stdout=tl,stderr=subprocess.STDOUT,env=env)
                    print(f'SCROBOT Odometry Lab: {run_dir}',flush=True)
                    while not stop_requested and runner.poll() is None and monitor.poll() is None:
                        time.sleep(.2)
                    if monitor.poll() is not None and runner.poll() is None:
                        print('Monitor stopped unexpectedly; interrupting motion.',file=sys.stderr)
                        stop_requested=True
                    if stop_requested and runner.poll() is None:
                        runner.send_signal(signal.SIGINT)
                        try:runner.wait(timeout=3)
                        except subprocess.TimeoutExpired:runner.kill();runner.wait()
                        stop_motion()
                    elif runner.poll() is not None and runner.returncode != 0:
                        stop_motion()
                    if not stop_requested and runner.returncode==0:
                        # The test runner publishes DONE and stops; leave a short tail.
                        time.sleep(1.5)
            else:
                print(f'SCROBOT passive monitoring: {run_dir}',flush=True)
                while not stop_requested and monitor.poll() is None:
                    time.sleep(.2)
    finally:
        if load is not None and load.poll() is None:
            load.terminate()
            try:load.wait(timeout=2)
            except subprocess.TimeoutExpired:load.kill();load.wait()
        if monitor is not None and monitor.poll() is None:
            monitor.send_signal(signal.SIGINT)
            try:monitor.wait(timeout=4)
            except subprocess.TimeoutExpired:monitor.kill();monitor.wait()
        if runner is not None and runner.poll() is None:
            runner.send_signal(signal.SIGINT)
            try:runner.wait(timeout=3)
            except subprocess.TimeoutExpired:runner.kill();runner.wait()
            stop_motion()
        if runner is not None and stop_requested:
            stop_motion()
        try:
            from analyze import analyze
            metrics=analyze(run_dir)
            print(f"Saved analysis: {run_dir/'summary.json'}",flush=True)
            print(json.dumps(metrics.get('accuracy',{}),indent=2),flush=True)
        except (FileNotFoundError,ValueError) as e:
            print(f'Analysis unavailable: {e}',file=sys.stderr,flush=True)
    if stop_requested:return 130
    if monitor is not None and monitor.returncode not in (0,None):return monitor.returncode
    return runner.returncode if runner else 0


if __name__=='__main__':
    sys.exit(main())
