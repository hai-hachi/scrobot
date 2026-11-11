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
        with urllib.request.urlopen(root+'/api/health') as f:
            assert json.load(f) == {'ok': True, 'service': 'scrobot_odometry_lab'}
        # Regression for the dashboard "Server disconnected" crash:
        # when a launcher log exists, /api/status must tail it without
        # calling nonexistent .stat() on its buffered file object.
        log_path = tmp_path / 'existing.launcher.log'
        log_path.write_text('old text' + 'x' * 6000 + 'latest log line', encoding='utf-8')
        service.run_log = log_path
        with urllib.request.urlopen(root+'/api/status') as f:
            status = json.load(f)
        assert status['log_tail'].endswith('latest log line')
        assert len(status['log_tail']) <= 2000
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

def test_motion_preflight_rejects_missing_controller_and_duplicate_clock(monkeypatch):
    """Lighting-only Gazebo, incomplete control stack and multiple clocks are unsafe."""
    import subprocess
    import web_server

    topics = BASE['topics']
    expected = ['/clock', topics['ground_truth'], topics['joint_states'],
                topics['wheel_odom'], topics['ekf_odom']]

    def check_case(*, missing=None, duplicate_clock=False, service=True):
        calls = []
        def fake_run(cmd, **kwargs):
            calls.append(cmd)
            if cmd[1:3] == ['topic', 'list']:
                listed = [t for t in expected if t != missing]
                out = '\n'.join(listed)
            elif cmd[1:3] == ['topic', 'info']:
                count = 2 if cmd[3] == '/clock' and duplicate_clock else 1
                out = f'Publisher count: {count}\nSubscription count: 1\n'
            elif cmd[1:3] == ['service', 'type']:
                out = 'std_srvs/srv/SetBool\n' if service else ''
            else:
                raise AssertionError(cmd)
            return subprocess.CompletedProcess(cmd, 0, stdout=out, stderr='')
        monkeypatch.setattr(web_server.subprocess, 'run', fake_run)
        return calls

    check_case(missing=topics['wheel_odom'])
    with pytest.raises(RuntimeError, match=topics['wheel_odom']):
        web_server.simulated_topics_present(BASE)

    check_case(duplicate_clock=True)
    with pytest.raises(RuntimeError, match='exactly one publisher on /clock'):
        web_server.simulated_topics_present(BASE)

    check_case(service=False)
    with pytest.raises(RuntimeError, match='set_manual_mode'):
        web_server.simulated_topics_present(BASE)

    calls = check_case()
    web_server.simulated_topics_present(BASE)
    assert ['ros2', 'service', 'type', '/control/set_manual_mode'] in calls


def test_slow_ros_preflight_does_not_block_web_status(tmp_path, monkeypatch):
    """A slow Gazebo/ROS discovery must not make the web server look disconnected."""
    import time
    import web_server
    cfg = deepcopy(BASE)
    cfg['run_root'] = str(tmp_path / 'runs')
    config_path = tmp_path / 'lab.yaml'
    config_path.write_text(yaml.safe_dump(cfg))
    service = LabService(config_path)
    checking = threading.Event()
    release = threading.Event()
    errors = []

    def slow_preflight(_):
        checking.set()
        assert release.wait(timeout=4), 'preflight test never released'
        raise RuntimeError('Gazebo unavailable after preflight')

    monkeypatch.setattr(web_server, 'simulated_topics_present', slow_preflight)
    worker = threading.Thread(
        target=lambda: _run_and_capture(service.start, errors),
        daemon=True,
    )
    worker.start()
    try:
        assert checking.wait(timeout=2), 'ROS preflight did not begin'
        t0 = time.monotonic()
        status = service.status()
        elapsed = time.monotonic() - t0
        assert elapsed < 0.5, 'Slow ROS preflight blocked dashboard status'
        assert status['preflight_in_progress'] is True
        assert status['running'] is False
        with pytest.raises(RuntimeError, match='already running or being checked'):
            service.start()
    finally:
        release.set()
        worker.join(timeout=3)
    assert not worker.is_alive()
    assert len(errors) == 1
    assert 'Gazebo unavailable' in str(errors[0])
    assert service.preflight_in_progress is False
    assert service.process is None


def _run_and_capture(call, errors):
    try:
        call()
    except Exception as exc:
        errors.append(exc)
