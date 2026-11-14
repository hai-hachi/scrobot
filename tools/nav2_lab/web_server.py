#!/usr/bin/env python3
"""Local-only Nav2 Lab browser service with isolated YAML and subprocesses."""
import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid
import webbrowser
from copy import deepcopy
from datetime import datetime
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse

import yaml
from core import PLANNERS, CONTROLLERS, build_nav2_params, validate_config

HERE = Path(__file__).resolve().parent
CONFIG = HERE / 'config' / 'lab.yaml'
WEB = HERE / 'web'
LAUNCH = HERE / 'launch' / 'nav2_lab.launch.py'
REPO = HERE.parents[1]


def production_params():
    # Repo layout is <workspace>/{src,tools}; never modify production YAML.
    source = REPO / 'src' / 'scrobot_navigation' / 'config' / 'nav2_params.yaml'
    if source.is_file():
        return yaml.safe_load(source.read_text())
    info = subprocess.run(['ros2', 'pkg', 'prefix', '--share', 'scrobot_navigation'],
                          capture_output=True, text=True, check=True, timeout=8)
    source = Path(info.stdout.strip()) / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(source.read_text())


def tail(path, limit=2400):
    if not path or not path.exists():
        return ''
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size - limit))
        return stream.read().decode('utf-8', 'replace')


def json_read(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, ValueError):
        return None


def available_nav2_plugins(env):
    required = {
        'nav2_planner', 'nav2_controller', 'nav2_navfn_planner',
        'nav2_smac_planner', 'nav2_theta_star_planner',
        'nav2_regulated_pure_pursuit_controller', 'nav2_dwb_controller',
        'nav2_mppi_controller', 'nav2_lifecycle_manager'
    }
    try:
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True,
                                text=True, timeout=12, check=False, env=env)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('ROS package discovery timed out') from exc
    if result.returncode:
        raise RuntimeError('Could not inspect installed ROS packages')
    missing = sorted(required - set(result.stdout.splitlines()))
    if missing:
        raise RuntimeError('Missing Nav2 ROS packages: ' + ', '.join(missing) +
                           '. Install ros-jazzy-navigation2 and source ROS 2.')


def existing_ros_graph(env):
    if not shutil.which('ros2'):
        raise RuntimeError('ROS 2 not found. Source /opt/ros/jazzy/setup.bash and workspace/install/setup.bash.')
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True,
                                text=True, timeout=8, check=False, env=env)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('ROS topic discovery timed out') from exc
    if result.returncode:
        raise RuntimeError('ROS 2 topic discovery failed. Check sourced ROS environment.')
    return set(result.stdout.splitlines())


def signal_group(process, sig):
    if process and process.poll() is None:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass


class Service:
    def __init__(self, config_path=CONFIG):
        self.config_path = Path(config_path)
        self.lock = threading.RLock()
        self.stack = None
        self.run = None
        self.stack_log = None
        self.run_log = None
        self.last_run = None
        self.active_params = None
        self.stack_env = None
        self.last_error = ''

    def config(self):
        config = yaml.safe_load(self.config_path.read_text())
        return validate_config(config)

    def save(self, payload):
        with self.lock:
            if not isinstance(payload, dict):
                raise ValueError('Expected JSON object')
            old = self.config()
            cfg = deepcopy(old)
            # Only accept exposed keys. Never accept arbitrary paths or topics.
            for key in ('planner', 'controller', 'mode'):
                if key in payload:
                    cfg[key] = payload[key]
            for group, fields in {
                'goal': ('x', 'y', 'yaw_deg'),
                'limits': ('cruise_speed',),
                'experiment': ('timeout_s', 'sample_hz'),
                'simulation': ('start_x', 'start_y', 'start_yaw', 'use_magnetometer'),
            }.items():
                if group in payload:
                    if not isinstance(payload[group], dict):
                        raise ValueError(f'{group} must be an object')
                    for field in fields:
                        if field in payload[group]:
                            cfg[group][field] = payload[group][field]
            validate_config(cfg)
            tmp = self.config_path.with_suffix('.tmp')
            tmp.write_text(yaml.safe_dump(cfg, sort_keys=False))
            os.replace(tmp, self.config_path)
            return cfg

    def active(self, process):
        return process is not None and process.poll() is None

    def root(self):
        return Path(self.config()['experiment']['run_root']).expanduser()

    def lab_env(self):
        env = os.environ.copy()
        domain = self.config()['simulation']['ros_domain_id']
        env['ROS_DOMAIN_ID'] = str(domain)
        env['GZ_PARTITION'] = f'scrobot_nav2_lab_{domain}'
        return env

    def start_stack(self):
        with self.lock:
            if self.active(self.stack):
                raise RuntimeError('Lab stack already running')
            if self.active(self.run):
                raise RuntimeError('Stop active experiment before restarting stack')
            lab_env = self.lab_env()
            available_nav2_plugins(lab_env)
            topics = existing_ros_graph(lab_env)
            if '/clock' in topics or '/joint_states' in topics or '/diff_drive_controller/odom' in topics:
                raise RuntimeError('Robot or simulation topics already exist in the Nav2 Lab ROS domain. Stop them or choose another isolated domain ID.')
            cfg = self.config()
            root = self.root()
            root.mkdir(parents=True, exist_ok=True)
            params = build_nav2_params(production_params(), cfg)
            self.active_params = root / '_nav2_lab_active.yaml'
            self.active_params.write_text(yaml.safe_dump(params, sort_keys=False))
            self.stack_log = root / '_nav2_lab_stack.log'
            args = [
                'ros2', 'launch', str(LAUNCH),
                'nav2_params_file:=' + str(self.active_params),
                'start_x:=' + str(cfg['simulation']['start_x']),
                'start_y:=' + str(cfg['simulation']['start_y']),
                'start_yaw:=' + str(cfg['simulation']['start_yaw']),
                'use_magnetometer:=' + str(cfg['simulation']['use_magnetometer']).lower()
            ]
            with self.stack_log.open('w') as log:
                self.stack = subprocess.Popen(
                    args, cwd=str(REPO), stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True, env=lab_env)
            self.stack_env = lab_env
            self.last_error = ''
            return self.status()

    def run_trial(self):
        with self.lock:
            if not self.active(self.stack):
                raise RuntimeError('Start the isolated Nav2 stack first')
            if self.active(self.run):
                raise RuntimeError('An experiment is already running')
            cfg = self.config()
            loaded = yaml.safe_load(self.active_params.read_text())
            active_speed = loaded['controller_server']['ros__parameters']['RPP']['desired_linear_vel']
            if abs(active_speed - cfg['limits']['cruise_speed']) > 1e-9:
                raise RuntimeError('Cruise speed changed since Nav2 launch. Stop and restart the lab simulation before benchmarking.')
            root = self.root()
            root.mkdir(parents=True, exist_ok=True)
            run_dir = root / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' +
                              cfg['planner'] + '_' + cfg['controller'] + '_' +
                              uuid.uuid4().hex[:5])
            run_dir.mkdir()
            snapshot = run_dir / 'config_snapshot.yaml'
            snapshot.write_text(yaml.safe_dump(cfg, sort_keys=False))
            shutil.copyfile(self.active_params, run_dir / 'nav2_params_snapshot.yaml')
            self.last_run = run_dir
            self.run_log = run_dir / 'runner.log'
            args = [sys.executable, str(HERE / 'runner.py'),
                    '--config', str(snapshot), '--output', str(run_dir)]
            with self.run_log.open('w') as log:
                self.run = subprocess.Popen(args, cwd=str(HERE),
                    stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True, env=self.stack_env)
            return self.status()

    def stop_trial(self):
        with self.lock:
            signal_group(self.run, signal.SIGINT)
            return self.status()

    def stop_stack(self):
        with self.lock:
            signal_group(self.run, signal.SIGINT)
            if self.active(self.run):
                try:
                    self.run.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    signal_group(self.run, signal.SIGTERM)
            signal_group(self.stack, signal.SIGINT)
            return self.status()

    def status(self):
        with self.lock:
            summary = json_read(self.last_run / 'summary.json') if self.last_run else None
            path = json_read(self.last_run / 'path.json') if self.last_run else None
            tracked = []
            if self.last_run and (self.last_run/'odometry.csv').exists():
                import csv
                with (self.last_run/'odometry.csv').open() as stream:
                    tracked = [[float(row['x_m']), float(row['y_m'])]
                               for row in csv.DictReader(stream)][::2][:1500]
            truth_aligned = []
            if self.last_run and (self.last_run/'ground_truth_aligned.csv').exists():
                import csv
                with (self.last_run/'ground_truth_aligned.csv').open() as stream:
                    truth_aligned = [[float(row['x_odom_m']), float(row['y_odom_m'])]
                                     for row in csv.DictReader(stream)][::2][:1500]
            return {
                'ground_truth_aligned': truth_aligned,
                'ros_domain_id': self.config()['simulation']['ros_domain_id'],
                'stack_running': self.active(self.stack),
                'stack_exit': self.stack.poll() if self.stack else None,
                'experiment_running': self.active(self.run),
                'experiment_exit': self.run.poll() if self.run else None,
                'run_name': self.last_run.name if self.last_run else None,
                'summary': summary, 'path': path, 'tracked': tracked,
                'stack_log_tail': tail(self.stack_log),
                'runner_log_tail': tail(self.run_log),
                'error': self.last_error,
                'loaded_speed_mps': (
                    yaml.safe_load(self.active_params.read_text())
                    ['controller_server']['ros__parameters']['RPP']['desired_linear_vel']
                    if self.active(self.stack) and self.active_params and self.active_params.exists()
                    else None),
            }

    def recent_runs(self):
        root = self.root()
        if not root.exists():
            return []
        runs = []
        for p in sorted(root.glob('*/summary.json'), reverse=True)[:30]:
            s = json_read(p)
            if s:
                runs.append({'name': p.parent.name, 'summary': s})
        return runs


class Handler(BaseHTTPRequestHandler):
    service = None

    def reply(self, obj, status=200):
        raw = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path == '/api/config':
                return self.reply(self.service.config())
            if path == '/api/status':
                return self.reply(self.service.status())
            if path == '/api/runs':
                return self.reply(self.service.recent_runs())
            filename = {'/':'index.html', '/index.html':'index.html',
                        '/app.js':'app.js', '/style.css':'style.css'}.get(path)
            if filename is None:
                return self.reply({'error':'Not found'}, 404)
            data = (WEB/filename).read_bytes()
            content_type = {'html':'text/html', 'js':'application/javascript',
                            'css':'text/css'}[filename.rsplit('.',1)[-1]]
            self.send_response(200)
            self.send_header('Content-Type', content_type+'; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)
        except Exception as exc:
            self.reply({'error':str(exc)}, 500)

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in ('/api/save', '/api/start-stack', '/api/stop-stack',
                        '/api/run', '/api/cancel'):
            return self.reply({'error': 'Not found'}, 404)
        try:
            origin = self.headers.get('Origin', '')
            if origin and urlparse(origin).hostname not in ('127.0.0.1', 'localhost'):
                return self.reply({'error': 'Cross-origin control requests forbidden'}, 403)
            if self.headers.get('Content-Type', '').split(';', 1)[0].strip() != 'application/json':
                return self.reply({'error': 'JSON Content-Type required'}, 415)
            length = int(self.headers.get('Content-Length','0'))
            if length > 16384:
                return self.reply({'error':'Request too large'},413)
            body = json.loads(self.rfile.read(length) or '{}')
            if path == '/api/save':
                out = {'config':self.service.save(body)}
            elif path == '/api/start-stack':
                out = self.service.start_stack()
            elif path == '/api/stop-stack':
                out = self.service.stop_stack()
            elif path == '/api/run':
                out = self.service.run_trial()
            else:
                out = self.service.stop_trial()
            return self.reply(out)
        except (ValueError, RuntimeError) as exc:
            self.reply({'error': str(exc)}, 400)
        except Exception as exc:
            self.reply({'error': str(exc)}, 500)

    def log_message(self, fmt, *args):
        # This service binds only to loopback and does not expose repo files.
        return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8772)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    Handler.service = Service()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print(f'Nav2 Lab web: http://127.0.0.1:{args.port}', flush=True)
    if not args.no_browser:
        webbrowser.open(f'http://127.0.0.1:{args.port}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        Handler.service.stop_stack()
        server.server_close()


if __name__ == '__main__':
    main()
