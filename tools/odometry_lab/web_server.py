#!/usr/bin/env python3
"""Local-only browser UI for the SCROBOT Odometry Lab.

Zero additional web framework dependencies. Motion can only be started after
verifying the simulated /clock and Gazebo ground-truth publishers exist.
"""
import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
import webbrowser
from copy import deepcopy
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import yaml
from storage import validated_result_root

HERE = Path(__file__).resolve().parent
CONFIG = HERE / 'config' / 'lab.yaml'
STATIC = HERE / 'web'
SCENARIOS = ('none', 'static', 'straight', 'rotate', 'arc', 'suite')
EXPORTS = {'summary.json': 'application/json', 'summary.csv': 'text/csv',
           'samples.csv': 'text/csv', 'wheel_speeds.csv': 'text/csv',
           'clock.csv': 'text/csv', 'trajectory.svg': 'image/svg+xml',
           'config_snapshot.yaml': 'text/yaml', 'launcher.log': 'text/plain',
           'monitor.log': 'text/plain', 'motion.log': 'text/plain'}


def validate_config(new, base):
    """Only allow editing known settings and enforce bounds server-side."""
    if not isinstance(new, dict):
        raise ValueError('Configuration must be a JSON object')
    out = deepcopy(base)
    # Paths, topics and unexposed settings are intentionally not writable from UI.
    limits = {
        'rates': {'encoder_sample_hz': (1, 500), 'estimator_hz': (1, 500),
                  'publish_hz': (1, 500), 'gui_hz': (1, 30)},
        'hardware': {
            'encoder_ticks_per_motor_rev': (1, 10000000),
            'gear_ratio': (.001, 10000), 'wheel_radius_m': (.001, 1),
            'wheel_separation_m': (.01, 5), 'left_radius_multiplier': (.1, 2),
            'right_radius_multiplier': (.1, 2),
            'encoder_noise_ticks_std': (0, 10000),
            'encoder_drop_probability': (0, 1),
            'feedback_delay_ms': (0, 10000), 'feedback_jitter_ms': (0, 10000)},
        'motion': {'command_publish_hz': (1, 100), 'linear_speed_mps': (.01, 1),
                   'straight_distance_m': (.05, 15), 'angular_speed_rps': (.01, 2),
                   'arc_linear_speed_mps': (.01, 1), 'arc_angular_speed_rps': (.01, 2),
                   'start_delay_s': (0, 30), 'stop_duration_s': (0, 20),
                   'static_duration_s': (0, 120), 'final_settle_duration_s': (0, 30)},
        'stress': {'cpu_duty': (0, .75)}, 'qos': {'depth': (1, 50)}
    }
    for group, fields in limits.items():
        src = new.get(group, {})
        if not isinstance(src, dict):
            raise ValueError(f'{group} must be an object')
        for key, (low, high) in fields.items():
            if key not in src:
                continue
            v = src[key]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f'{group}.{key} must be a finite number')
            if not low <= v <= high:
                raise ValueError(f'{group}.{key} must be between {low} and {high}')
            old = out[group][key]
            if isinstance(old, int):
                if int(v) != v:
                    raise ValueError(f'{group}.{key} must be a whole number')
                v = int(v)
            out[group][key] = v
    if 'qos' in new and 'reliability' in new['qos']:
        v = new['qos']['reliability']
        if v not in ('best_effort', 'reliable'):
            raise ValueError('qos.reliability must be best_effort or reliable')
        out['qos']['reliability'] = v
    if 'scenario' in new:
        if new['scenario'] not in SCENARIOS:
            raise ValueError('Unsupported motion scenario')
        out['scenario'] = new['scenario']
    return out


def tail_csv(path, limit=1200):
    """Efficiently return recent pose samples without rereading an entire run."""
    if not path.is_file():
        return []
    limit = max(10, min(3000, limit))
    chunks = []
    with path.open('rb') as f:
        f.seek(0, 2)
        pos = f.tell()
        while pos and len(b'\n'.join(reversed(chunks)).splitlines()) <= limit + 2:
            n = min(pos, 32768)
            pos -= n
            f.seek(pos)
            chunks.append(f.read(n))
            if sum(map(len, chunks)) > 3_000_000:
                break
    raw = b''.join(reversed(chunks)).decode('utf8', 'replace')
    lines = raw.splitlines()
    if pos:
        lines = lines[1:]
    data = []
    for line in lines[-limit:]:
        try:
            cells = next(csv.reader([line]))
            if len(cells) != 10 or cells[0] not in ('ground_truth', 'wheel_odom', 'ekf_odom', 'encoder_odom'):
                continue
            stamp, x, y, heading = [float(cells[i]) for i in (3, 4, 5, 6)]
            if all(math.isfinite(v) for v in (stamp, x, y, heading)):
                data.append([cells[0], stamp, x, y, heading])
        except (ValueError, csv.Error):
            continue
    return data


def safe_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def simulated_topics_present(config):
    """Require exactly one Gazebo clock/truth source and a live control stack.

    Lighting Lab and Mobility Lab normally launch independent Gazebo worlds.
    Never start a motion trial against either bare simulation, a real robot,
    or a ROS graph with multiple simulation clock publishers.
    """
    def ros_cli(*arguments, timeout=8):
        command = ['ros2', *arguments]
        try:
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=timeout, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f'ROS 2 discovery unavailable for {arguments}: {exc}') from exc
        if result.returncode:
            raise RuntimeError(
                f'ROS 2 discovery failed for {" ".join(arguments)}; '
                'source Jazzy and run the normal SCROBOT simulation stack.'
            )
        return result.stdout

    listed = set(ros_cli('topic', 'list').splitlines())
    topics = config['topics']
    truth = topics['ground_truth']
    required = ('/clock', truth, topics['joint_states'],
                topics['wheel_odom'], topics['ekf_odom'])
    for name in required:
        if name not in listed:
            raise RuntimeError(
                f'Required SCROBOT topic missing: {name}. Start the normal '
                'simulation/control/localization stack, not the isolated '
                'Lighting or Mobility Lab Gazebo.'
            )

    for name in required:
        information = ros_cli('topic', 'info', name, timeout=6)
        match = re.search(r'Publisher count:\s*(\d+)', information)
        if not match:
            raise RuntimeError(f'Cannot determine publisher count on {name}')
        count = int(match.group(1))
        if name in ('/clock', truth) and count != 1:
            raise RuntimeError(
                f'Expected exactly one publisher on {name}, found {count}. '
                'Stop duplicate Gazebo simulations/bridges before motion.'
            )
        if count == 0:
            raise RuntimeError(
                f'No active publisher on {name}. Launch the normal SCROBOT '
                'simulation, controllers and localization first.'
            )

    service_type = ros_cli('service', 'type', '/control/set_manual_mode',
                           timeout=6).strip()
    if service_type != 'std_srvs/srv/SetBool':
        raise RuntimeError(
            'Missing /control/set_manual_mode (std_srvs/srv/SetBool). '
            'Odometry motion requires the production manual/safety pipeline.'
        )


class LabService:
    def __init__(self, config_path=CONFIG):
        self.config_path = Path(config_path)
        self.lock = threading.RLock()
        self.process = None
        self.run_dir = None
        self.run_log = None
        self.last_error = ''
        self.run_started_wall = None
        self.preflight_in_progress = False

    def config(self):
        return yaml.safe_load(self.config_path.read_text())

    def apply(self, payload):
        with self.lock:
            cfg = validate_config(payload, self.config())
            # A user-edited config must not redirect results into the
            # Lighting/Mobility Lab trees or the source checkout.
            validated_result_root(cfg['run_root'])
            tmp = self.config_path.with_suffix('.yaml.tmp')
            tmp.write_text(yaml.safe_dump(cfg, sort_keys=False))
            os.replace(tmp, self.config_path)
            return cfg

    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self):
        # Do not hold self.lock during potentially slow ROS CLI discovery.
        # Otherwise status polling blocks and the dashboard appears offline.
        # Reserve the start operation first, so two tabs cannot start
        # competing motion experiments while preflight runs.
        with self.lock:
            if self.running() or self.preflight_in_progress:
                raise RuntimeError('A test is already running or being checked')
            cfg = self.config()
            scenario = cfg.get('scenario', 'none')
            if scenario not in SCENARIOS:
                raise RuntimeError('Invalid configured scenario')
            root = validated_result_root(cfg['run_root'])
            self.preflight_in_progress = True
        try:
            if scenario != 'none':
                simulated_topics_present(cfg)
            with self.lock:
                if self.running():
                    raise RuntimeError('A test is already running')
                if self.config() != cfg:
                    raise RuntimeError(
                        'Lab settings changed during ROS preflight; restart the experiment'
                    )
                root.mkdir(parents=True, exist_ok=True)
                name = 'web_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]
                run_dir = root / name
                run_log = root / (name + '.launcher.log')
                if run_dir.exists():
                    raise RuntimeError('Odometry run name already exists')
                try:
                    log = run_log.open('x')
                except FileExistsError as exc:
                    raise RuntimeError('Odometry launcher log already exists') from exc
                try:
                    process = subprocess.Popen(
                        [sys.executable, str(HERE / 'run_lab.py'),
                         '--config', str(self.config_path), '--scenario', scenario,
                         '--run-name', name],
                        cwd=str(HERE), stdout=log,
                        stderr=subprocess.STDOUT, start_new_session=True,
                    )
                finally:
                    log.close()
                # Assign current state only after successful process startup.
                self.process = process
                self.run_dir = run_dir
                self.run_log = run_log
                self.last_error = ''
                self.run_started_wall = time.monotonic()
        finally:
            with self.lock:
                self.preflight_in_progress = False
        return self.status()

    def stop(self):
        with self.lock:
            if self.running():
                # The launcher catches SIGINT and sends zero + AUTO restoration.
                self.process.send_signal(signal.SIGINT)
                return 'Stop requested; waiting for launcher safety cleanup.'
            return 'No test running.'

    def status(self):
        with self.lock:
            cfg = self.config()
            alive = self.running()
            latest = self.run_dir
            if latest is None:
                root = Path(cfg['run_root']).expanduser()
                if root.exists():
                    choices = [p for p in root.iterdir() if p.is_dir() and (p / 'metadata.json').exists()]
                    latest = max(choices, key=lambda p: p.stat().st_mtime) if choices else None
            live = safe_json(latest / 'live.json') if latest else None
            report = safe_json(latest / 'summary.json') if latest else None
            code = None if self.process is None else self.process.poll()
            logs = ''
            path = self.run_log
            if not path and latest:
                path = latest.parent / (latest.name + '.launcher.log')
            if path and path.exists():
                try:
                    with path.open('rb') as f:
                        # A buffered file has no .stat(). Seek from its own
                        # end to safely tail even while the launcher writes.
                        f.seek(0, os.SEEK_END)
                        f.seek(max(0, f.tell() - 4500), os.SEEK_SET)
                        logs = f.read().decode('utf8', 'replace')[-2000:]
                except OSError:
                    pass
            return {'running': alive, 'preflight_in_progress': self.preflight_in_progress,
                    'returncode': code,
                    'run_name': latest.name if latest else None,
                    'run_dir': str(latest) if latest else None,
                    'live': live, 'summary': report, 'log_tail': logs,
                    'scenario': cfg.get('scenario'), 'error': self.last_error,
                    'run_elapsed_wall_s': (time.monotonic() - self.run_started_wall)
                    if alive and self.run_started_wall else None}

    def samples(self, limit):
        with self.lock:
            return tail_csv(self.run_dir / 'samples.csv', limit) if self.run_dir else []

    def export_path(self, filename):
        if filename not in EXPORTS or self.run_dir is None:
            raise FileNotFoundError('No export available')
        p = self.run_dir / filename
        if not p.is_file():
            raise FileNotFoundError(filename)
        return p

    def shutdown(self):
        self.stop()
        with self.lock:
            if self.running():
                try:
                    self.process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    # Do not claim safety cleanup succeeded if launcher is unresponsive.
                    print('WARNING: Odometry Lab launcher did not finish stop cleanup.', file=sys.stderr)


class Handler(BaseHTTPRequestHandler):
    service = None
    def log_message(self, format, *args):
        return

    def send(self, data, status=200, ctype='application/json'):
        payload = json.dumps(data, allow_nan=False).encode() if ctype == 'application/json' else data
        self.send_response(status)
        self.send_header('Content-Type', ctype + ('; charset=utf-8' if ctype.startswith('text/') or ctype == 'application/json' else ''))
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        query = urlsplit(self.path)
        route = query.path
        try:
            if route == '/api/health':
                # Fast HTTP-only connectivity check, independent of Gazebo/ROS.
                return self.send({'ok': True, 'service': 'scrobot_odometry_lab'})
            if route == '/api/config':
                return self.send(self.service.config())
            if route == '/api/status':
                return self.send(self.service.status())
            if route == '/api/samples':
                n = int(parse_qs(query.query).get('limit', ['1600'])[0]); return self.send({'samples': self.service.samples(n)})
            if route == '/api/export':
                name = parse_qs(query.query).get('file', [''])[0]
                path = self.service.export_path(name)
                return self.send(path.read_bytes(), ctype=EXPORTS[name])
            static = {'/': ('index.html', 'text/html'), '/app.js': ('app.js', 'application/javascript'),
                      '/style.css': ('style.css', 'text/css')}
            if route in static:
                filename, ctype = static[route]
                return self.send((STATIC / filename).read_bytes(), ctype=ctype)
            return self.send({'error': 'Not found'}, 404)
        except (ValueError, OSError, FileNotFoundError) as exc:
            return self.send({'error': str(exc)}, 400)

    def do_POST(self):
        try:
            host = self.headers.get('Host', '')
            origin = self.headers.get('Origin', '')
            if host not in ('127.0.0.1:' + str(self.server.server_port),
                            'localhost:' + str(self.server.server_port)):
                return self.send({'error': 'Localhost access only'}, 403)
            if origin and origin not in ('http://' + host,):
                return self.send({'error': 'Cross-origin request denied'}, 403)
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                return self.send({'error': 'JSON required'}, 415)
            n = int(self.headers.get('Content-Length', '0'))
            if not 0 <= n <= 128_000:
                return self.send({'error': 'Request body too large'}, 413)
            payload = json.loads(self.rfile.read(n) or b'{}')
            if self.path == '/api/apply':
                return self.send({'config': self.service.apply(payload), 'message': 'Settings saved; current run unchanged'})
            if self.path == '/api/start':
                return self.send({'status': self.service.start()})
            if self.path == '/api/stop':
                return self.send({'message': self.service.stop()})
            return self.send({'error': 'Not found'}, 404)
        except (RuntimeError, ValueError, OSError, KeyError, TypeError) as exc:
            return self.send({'error': str(exc)}, 400)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8767)
    ap.add_argument('--no-browser', action='store_true')
    args = ap.parse_args(argv)
    if not 1024 <= args.port <= 65535:
        ap.error('Port must be 1024..65535')
    service = LabService()
    Handler.service = service
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    url = f'http://127.0.0.1:{args.port}'
    print(f'SCROBOT Odometry Lab Web: {url}', flush=True)
    if not args.no_browser:
        threading.Timer(.7, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=.3)
    except KeyboardInterrupt:
        pass
    finally:
        service.shutdown()
        server.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
