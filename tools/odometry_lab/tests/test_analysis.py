import csv
import math
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze import calculate,analyze


def test_start_frame_alignment():
    # Truth is world-shifted and rotated relative to wheel odometry.
    gt=[(i,5.,-2.+.1*i,math.pi/2, .1,0.,'STRAIGHT_FWD') for i in range(1,11)]
    odom=[(i,.1*(i-1),0.,0.,.1,0.,'STRAIGHT_FWD') for i in range(1,11)]
    metrics=calculate(gt,{'wheel_odom':odom})['wheel_odom']
    assert metrics['position_rmse_m']<1e-10
    assert metrics['heading_rmse_deg']<1e-10


def test_outlier_and_duplicate_stamps(tmp_path):
    fields=['stream','wall_time_s','sim_recv_s','stamp_s','x_m','y_m','yaw_rad','vx_mps','wz_rps','state']
    with (tmp_path/'samples.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for k in range(1,11):
            for stream,scale in [('ground_truth',1.),('wheel_odom',1.1)]:
                writer.writerow({'stream':stream,'wall_time_s':k/10,'sim_recv_s':k+.1,
                    'stamp_s':k,'x_m':k*.1*scale,'y_m':0,'yaw_rad':0,
                    'vx_mps':.1*scale,'wz_rps':0,'state':'STRAIGHT_FWD'})
    report=analyze(tmp_path,plots=False)
    assert report['accuracy']['wheel_odom']['position_rmse_m']>.03
    assert (tmp_path/'summary.json').exists()
    assert (tmp_path/'summary.csv').exists()
