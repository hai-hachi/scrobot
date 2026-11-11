"""Web dashboard tests require no ROS 2/Gazebo installation."""
import json
import sys
import threading
import urllib.error
import urllib.request
from copy import deepcopy
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web_server import LabService, Handler, tail_csv, validate_config

ROOT = Path(__file__).resolve().parents[1]
BASE = yaml.safe_load((ROOT / 'config/lab.yaml').read_text())


def test_validate_accepts_rate_and_scenario():
    cfg = validate_config({'rates': {'publish_hz': 100}, 'scenario': 'rotate',
                           'qos': {'reliability': 'reliable'}}, BASE)
    assert cfg['rates']['publish_hz'] == 100
    assert cfg['scenario'] == 'rotate'
    assert cfg['qos']['reliability'] == 'reliable'
    assert BASE['rates']['publish_hz'] == 50


@pytest.mark.parametrize('bad', [
    {'rates': {'publish_hz': -2}}, {'rates': {'publish_hz': '100'}},
    {'hardware': {'encoder_ticks_per_motor_rev': 1.5}},
    {'hardware': {'wheel_radius_m': 0}},
    {'stress': {'cpu_duty': 1}}, {'qos': {'reliability': 'sometimes'}},
    {'scenario': '../../bad'}, {'rates': {'publish_hz': float('nan')}},
])
def test_validate_rejects_invalid_parameters(bad):
    with pytest.raises(ValueError):
        validate_config(bad, BASE)


def test_web_config_does_not_change_production_topics():
    cfg=validate_config({'topics': {'wheel_odom': '/other'},
                         'run_root': '/tmp/another',
                         'hardware': {'feedback_delay_ms': 20}}, BASE)
    assert cfg['topics'] == BASE['topics']
    assert cfg['run_root'] == BASE['run_root']
    assert cfg['hardware']['feedback_delay_ms'] == 20


def test_tail_csv_recent_lines(tmp_path):
    p=tmp_path / 'samples.csv'
    p.write_text('stream,wall_time_s,sim_recv_s,stamp_s,x_m,y_m,yaw_rad,vx_mps,wz_rps,state\n'
                 + ''.join(f'wheel_odom,{i},{i},{i},{i},0,0,0,0,STRAIGHT\n' for i in range(100)))
    data=tail_csv(p,10)
    assert len(data)==10
    assert data[0][2]==90 and data[-1][2]==99


def test_web_http_and_local_apply(tmp_path):
    cfg=deepcopy(BASE)
    cfg['run_root']=str(tmp_path / 'runs')
    yamlpath=tmp_path / 'lab.yaml'
    yamlpath.write_text(yaml.safe_dump(cfg))
    service=LabService(yamlpath)
    class LocalHandler(Handler):
        pass
    LocalHandler.service=service
    http=ThreadingHTTPServer(('127.0.0.1',0), LocalHandler)
    thread=threading.Thread(target=http.serve_forever,daemon=True)
    thread.start()
    root=f'http://127.0.0.1:{http.server_port}'
    try:
        with urllib.request.urlopen(root+'/') as f:
            assert b'Odometry' in f.read()
        with urllib.request.urlopen(root+'/app.js') as f:
            assert b'function plotTrajectory' in f.read()
        with urllib.request.urlopen(root+'/api/config') as f:
            assert json.load(f)['scenario'] == 'straight'
        content=json.dumps({'rates':{'publish_hz':100},'scenario':'none'}).encode()
        req=urllib.request.Request(root+'/api/apply',data=content,
                    headers={'Content-Type':'application/json','Origin':root},method='POST')
        with urllib.request.urlopen(req) as f:
            assert json.load(f)['config']['rates']['publish_hz']==100
        assert yaml.safe_load(yamlpath.read_text())['scenario']=='none'
        evil=urllib.request.Request(root+'/api/stop', data=b'{}',
                    headers={'Content-Type':'application/json','Origin':'https://example.com'}, method='POST')
        with pytest.raises(urllib.error.HTTPError) as exc:
            urllib.request.urlopen(evil)
        assert exc.value.code==403
    finally:
        http.shutdown();http.server_close();thread.join(timeout=5)


def test_motion_start_preflight_is_required(tmp_path, monkeypatch):
    cfg=deepcopy(BASE);cfg['run_root']=str(tmp_path / 'runs')
    f=tmp_path / 'lab.yaml';f.write_text(yaml.safe_dump(cfg))
    service=LabService(f)
    import web_server
    monkeypatch.setattr(web_server,'simulated_topics_present',lambda config: (_ for _ in ()).throw(RuntimeError('Gazebo offline')))
    with pytest.raises(RuntimeError,match='Gazebo offline'):
        service.start()
    assert service.process is None
