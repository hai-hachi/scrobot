#!/usr/bin/env python3
"""Tkinter Odometry Lab: configuration Apply saves YAML, tests run in subprocesses."""
import csv
import json
import subprocess
import sys
from collections import defaultdict,deque
from pathlib import Path
import math
import tkinter as tk
from tkinter import ttk,messagebox

import yaml

HERE=Path(__file__).resolve().parent
CONFIG=HERE/'config'/'lab.yaml'


class LabGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('SCROBOT | Odometry Lab')
        self.geometry('1120x730')
        self.config_data=yaml.safe_load(CONFIG.read_text())
        self.variables={}
        self.process=None
        self.run_dir=None
        self.run_root=Path(self.config_data['run_root']).expanduser()
        self._make_ui()
        self.protocol('WM_DELETE_WINDOW',self.close)
        self.after(800,self.refresh)

    def variable(self,path):
        section,key=path.split('.')
        v=tk.StringVar(value=str(self.config_data[section][key]))
        self.variables[path]=v
        return v

    def field(self,parent,label,path,width=14):
        f=ttk.Frame(parent);f.pack(fill='x',pady=3)
        ttk.Label(f,text=label,width=26).pack(side='left')
        ttk.Entry(f,textvariable=self.variable(path),width=width).pack(side='left')

    def _make_ui(self):
        head=ttk.Frame(self,padding=10);head.pack(fill='x')
        ttk.Label(head,text='SCROBOT  /  ODOMETRY LAB',font=('Sans',16,'bold')).pack(side='left')
        ttk.Label(head,text='Production-safe isolated odometry topic | Gazebo must already be running').pack(side='right')
        notebook=ttk.Notebook(self);notebook.pack(fill='both',expand=True,padx=10,pady=5)
        self.tabs={name:ttk.Frame(notebook,padding=12) for name in ('Timing','Performance','Hardware','Accuracy')}
        for name,frame in self.tabs.items():notebook.add(frame,text=name)
        p=self.tabs['Timing']
        for label,path in [('Encoder sampling (Hz)','rates.encoder_sample_hz'),
            ('Estimation (Hz)','rates.estimator_hz'),('Lab topic publish (Hz)','rates.publish_hz'),
            ('GUI refresh (Hz)','rates.gui_hz')]:self.field(p,label,path)
        self.field(p,'Motion command rate (Hz)','motion.command_publish_hz')
        self.field(p,'Lab QoS depth','qos.depth')
        f=ttk.Frame(p);f.pack(fill='x',pady=3)
        ttk.Label(f,text='Lab QoS reliability',width=26).pack(side='left')
        rel=tk.StringVar(value=self.config_data.get('qos',{}).get('reliability','best_effort'))
        self.variables['qos.reliability']=rel
        ttk.Combobox(f,textvariable=rel,state='readonly',width=15,values=['best_effort','reliable']).pack(side='left')
        self.timing_text=tk.Text(p,height=15,width=90,state='disabled');self.timing_text.pack(fill='both',expand=True,pady=10)
        p=self.tabs['Performance']
        self.field(p,'Synthetic CPU duty (0..0.75)','stress.cpu_duty')
        self.perf=tk.StringVar(value='No live metrics yet')
        ttk.Label(p,textvariable=self.perf,font=('Monospace',12),justify='left').pack(anchor='nw')
        ttk.Label(p,text='RTF is derived from /clock versus monotonic wall time; CPU is this lab process only.').pack(anchor='nw',pady=12)
        p=self.tabs['Hardware']
        for label,path in [('Motor encoder ticks/rev','hardware.encoder_ticks_per_motor_rev'),
            ('Motor / wheel gear ratio','hardware.gear_ratio'),
            ('Wheel radius (m)','hardware.wheel_radius_m'),
            ('Wheel separation (m)','hardware.wheel_separation_m'),
            ('Left radius multiplier','hardware.left_radius_multiplier'),
            ('Right radius multiplier','hardware.right_radius_multiplier'),
            ('Encoder noise (ticks std)','hardware.encoder_noise_ticks_std'),
            ('Drop probability (0..1)','hardware.encoder_drop_probability'),
            ('Feedback delay (ms)','hardware.feedback_delay_ms'),
            ('Feedback jitter (ms)','hardware.feedback_jitter_ms')]:self.field(p,label,path)
        ttk.Label(p,text='Encoder model reads /joint_states; does not replace existing /odom or publish TF.').pack(anchor='w',pady=8)
        p=self.tabs['Accuracy']
        self.canvas=tk.Canvas(p,bg='white',height=390);self.canvas.pack(fill='both',expand=True)
        self.accuracy=tk.StringVar(value='No recorded trajectory yet')
        ttk.Label(p,textvariable=self.accuracy,justify='left').pack(anchor='w',pady=5)
        foot=ttk.Frame(self,padding=10);foot.pack(fill='x')
        self.scenario=tk.StringVar(value=self.config_data.get('scenario','straight'))
        ttk.Label(foot,text='Scenario').pack(side='left')
        ttk.Combobox(foot,textvariable=self.scenario,state='readonly',width=12,
            values=['none','static','straight','rotate','arc','suite']).pack(side='left',padx=6)
        ttk.Button(foot,text='Apply → save YAML',command=self.apply).pack(side='left',padx=5)
        ttk.Button(foot,text='Start',command=self.start).pack(side='left',padx=5)
        ttk.Button(foot,text='Stop',command=self.stop).pack(side='left',padx=5)
        ttk.Button(foot,text='Open Results',command=self.show_results).pack(side='left',padx=5)
        self.status=tk.StringVar(value='Idle; start the Gazebo simulation and control stack first')
        ttk.Label(self,textvariable=self.status,padding=(10,3)).pack(fill='x')

    def apply(self):
        cfg=yaml.safe_load(CONFIG.read_text())
        try:
            for path,var in self.variables.items():
                sec,key=path.split('.')
                orig=cfg[sec][key]
                value=var.get().strip()
                cfg[sec][key]=value if isinstance(orig,str) else (int(value) if isinstance(orig,int) else float(value))
            for key in ('encoder_sample_hz','estimator_hz','publish_hz','gui_hz'):
                if not 1<=cfg['rates'][key]<=500:raise ValueError(f'{key} must be 1..500 Hz')
            if not 1<=cfg['motion']['command_publish_hz']<=100:raise ValueError('Motion command rate must be 1..100 Hz')
            if not 1<=cfg['qos']['depth']<=50:raise ValueError('QoS depth must be 1..50')
            if cfg['qos']['reliability'] not in ('best_effort','reliable'):
                raise ValueError('QoS reliability must be best_effort/reliable')
            if not 0<=cfg['stress']['cpu_duty']<=.75:raise ValueError('CPU stress duty must be 0..0.75')
            hw=cfg['hardware']
            if min(hw['wheel_radius_m'],hw['wheel_separation_m'],hw['gear_ratio'],hw['encoder_ticks_per_motor_rev'],
                   hw['left_radius_multiplier'],hw['right_radius_multiplier'])<=0:raise ValueError('Geometry and encoder values must be positive')
            if not 0<=hw['encoder_drop_probability']<=1:raise ValueError('Drop probability must be 0..1')
            if min(hw['encoder_noise_ticks_std'],hw['feedback_delay_ms'],hw['feedback_jitter_ms'])<0:
                raise ValueError('Noise, delay and jitter must be nonnegative')
            cfg['scenario']=self.scenario.get()
            CONFIG.write_text(yaml.safe_dump(cfg,sort_keys=False))
            self.config_data=cfg
            self.run_root=Path(cfg['run_root']).expanduser()
            self.status.set(f'Saved {CONFIG} (running tests are unaffected)')
            return True
        except (ValueError,TypeError) as e:
            messagebox.showerror('Invalid configuration',str(e))
            return False

    def start(self):
        if self.process and self.process.poll() is None:
            messagebox.showinfo('Already running','Stop the current run first.');return
        if not self.apply():return
        before=set(self.run_root.glob('*')) if self.run_root.exists() else set()
        self.run_dir=None
        self.process=subprocess.Popen([sys.executable,str(HERE/'run_lab.py'),
            '--config',str(CONFIG),'--scenario',self.scenario.get()],
            stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        self.status.set('Starting run; watch monitor.log and motion.log in results directory')
        # New run directory is created by the child process.
        self._known_dirs=before

    def stop(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()  # launcher interrupts runner and attempts a zero command / AUTO restore
            self.status.set('Stopping; test controller will be asked to stop')

    def close(self):
        self.stop()
        self.destroy()

    def show_results(self):
        if not self.run_dir:return
        summary=self.run_dir/'summary.json'
        if summary.exists():
            messagebox.showinfo('Analysis',summary.read_text()[:5000])
        else:
            messagebox.showinfo('Results',str(self.run_dir))

    def refresh(self):
        try:
            if self.process and self.run_dir is None and self.run_root.exists():
                created=[p for p in self.run_root.iterdir() if p.is_dir() and p not in getattr(self,'_known_dirs',set())]
                if created:self.run_dir=max(created,key=lambda p:p.stat().st_mtime)
            if self.process and self.process.poll() is not None:
                self.status.set(f'Run finished (exit {self.process.returncode}) | {self.run_dir}')
                self.process=None
            if self.run_dir:
                live=self.run_dir/'live.json'
                if live.exists():
                    data=json.loads(live.read_text())
                    factor=data.get('rtf');cpu=data.get('lab_cpu_percent')
                    self.perf.set(f"Gazebo RTF: {factor if factor is not None else 'N/A'}\n"
                        f"Lab process CPU %: {cpu if cpu is not None else 'N/A'}\n"
                        f"Lab memory MiB: {data.get('lab_memory_mib')}\n"
                        f"System CPU %: {data.get('system_cpu_percent')}\n"
                        f"Gazebo CPU % estimate: {data.get('gazebo_process_cpu_percent_estimate')}\n"
                        f"Callback runtimes (ms): {data.get('callback_runtime_ms')}\n"
                        f"Encoder updates: {data.get('encoder_updates')}\n"
                        f"Emulated delayed samples: {data.get('pending_samples')}\n"
                        f"Test phase: {data.get('state')}")
                    self.timing_text.configure(state='normal');self.timing_text.delete('1.0','end')
                    for key,val in data.get('timings',{}).items():
                        self.timing_text.insert('end',f"{key:16} {val.get('wall_hz',0):7.1f} Hz wall | "
                            f"{val.get('stamp_hz',0):7.1f} Hz timestamps | "
                            f"p95 period {val.get('wall_period_p95_ms',0):6.1f} ms | "
                            f"repeated {val.get('repeated_stamps',0)} | samples {val.get('count',0)}\n")
                    self.timing_text.configure(state='disabled')
                samples=self.run_dir/'samples.csv'
                if samples.exists():self.draw_trajectory(samples)
                summary=self.run_dir/'summary.json'
                if summary.exists():
                    acc=json.loads(summary.read_text()).get('accuracy',{})
                    self.accuracy.set(' | '.join(f"{k}: {v['position_rmse_m']:.3f} m, {v['heading_rmse_deg']:.2f}°"
                        for k,v in acc.items()) or 'No matched trajectory')
        except (OSError,ValueError,KeyError) as e:
            self.status.set(f'Monitoring data unavailable: {e}')
        finally:
            self.after(1000,self.refresh)

    def draw_trajectory(self,samples):
        # Read only a decimated recent window; GUI drawing never runs in ROS callbacks.
        data=defaultdict(lambda:deque(maxlen=1000))
        with open(samples,newline='') as f:
            for i,row in enumerate(csv.DictReader(f)):
                if i%5:continue
                try:data[row['stream']].append((float(row['x_m']),float(row['y_m']),float(row['yaw_rad'])))
                except (KeyError,ValueError):pass
        if not data:return
        # Align the first point and heading in each trajectory for display;
        # precise timestamp-matched comparison is performed by analyze.py.
        aligned={}
        for name,poses in data.items():
            if not poses:continue
            x0,y0,a0=poses[0]
            aligned[name]=[((x-x0)*math.cos(a0)+(y-y0)*math.sin(a0),
                            -(x-x0)*math.sin(a0)+(y-y0)*math.cos(a0))
                           for x,y,_ in poses]
        data=aligned
        coords=[p for stream in data.values() for p in stream]
        xmin=min(p[0] for p in coords);xmax=max(p[0] for p in coords)
        ymin=min(p[1] for p in coords);ymax=max(p[1] for p in coords)
        dx=max(1e-6,xmax-xmin);dy=max(1e-6,ymax-ymin)
        w=max(300,self.canvas.winfo_width());h=max(200,self.canvas.winfo_height())
        scale=min((w-50)/dx,(h-50)/dy)
        self.canvas.delete('all')
        colors={'ground_truth':'#238b66','wheel_odom':'#337ab7',
                'ekf_odom':'#9854bb','encoder_odom':'#d89022'}
        for i,(name,points) in enumerate(data.items()):
            if len(points)<2:continue
            # SE(2) aligned origin is computed in the offline report; this is raw live pose.
            flat=[]
            for x,y in points:flat += [25+(x-xmin)*scale,h-25-(y-ymin)*scale]
            self.canvas.create_line(*flat,fill=colors.get(name,'black'),width=2)
            self.canvas.create_text(15,12+i*18,anchor='w',text=name,fill=colors.get(name,'black'))


if __name__=='__main__':
    LabGUI().mainloop()
