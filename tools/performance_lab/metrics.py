"""Clock-aware topic rates: never mistake slow wall time for missing sim cycles.

Pure-Python core, no ROS or Gazebo imports, so behavior is testable offline.
Each topic event records both monotonic arrival time and the latest /clock
value. Header-based age and gaps are separate diagnostics; no claim of
exact native publisher rate follows from a ROS subscriber.
"""
from collections import deque
import math
import time


TOPIC_SPECS={
    "/camera/camera/color/image_raw":(30.0,"RGB image"),
    "/camera/camera/depth/image_raw":(30.0,"Depth image"),
    "/camera/camera/depth/points":(30.0,"Depth point cloud"),
    "/camera/camera/imu":(100.0,"D435i IMU"),
    "/imu/mag":(100.0,"Magnetometer"),
    "/diff_drive_controller/odom":(100.0,"Wheel odometry; controller proxy"),
    "/odometry/filtered":(100.0,"EKF odometry"),
    "/joint_states":(100.0,"Joint states; NOT PID callback rate"),
    "/camera/camera/depth/scan_raw":(30.0,"Cloud to scan"),
    "/camera/camera/depth/scan":(30.0,"Filtered scan"),
    "/global_costmap/costmap":(3.0,"Nav2 global costmap publication"),
    "/perception/shuttle_detections":(None,"YOLO detections if published"),
}


def stamp_seconds(stamp):
    if stamp is None:return None
    sec=float(stamp.sec)+float(stamp.nanosec)*1e-9
    return sec if math.isfinite(sec) and sec>0 else None


def frequency(items,index):
    if len(items)<2:return None
    delta=items[-1][index]-items[0][index]
    return round((len(items)-1)/delta,2) if delta>0 else None


def assess(row,rtf,paused,mode="synchronized"):
    """Only classify observable rates; never turn 'no data' into PASS."""
    if paused:return "paused"
    hz=row.get("sim_hz")
    expected=row.get("expected_sim_hz")
    if hz is None or expected is None:return "unknown"
    ratio=hz/expected
    if ratio<.90:return "sim_rate_low"
    if ratio>1.15:return "sim_rate_high"
    if mode=="real_time" and rtf is not None and rtf<.95:return "rtf_low"
    return "pass"


class ClockMeter:
    def __init__(self,topics=None,window_s=5.0):
        self.window_s=float(window_s)
        self.topic_specs=dict(topics or TOPIC_SPECS)
        self.clock_samples=deque(maxlen=2500)
        self.events={topic:deque(maxlen=3200) for topic in self.topic_specs}
        self.sim_time=None
        self.last_clock_wall=None
        self.jumps=0
        self.duplicate_clock=0

    def clock(self,sim,wall=None):
        wall=time.monotonic() if wall is None else float(wall)
        sim=float(sim)
        if not math.isfinite(sim):return
        if self.sim_time is not None and sim < self.sim_time-1e-6:
            self.jumps+=1
            self.clock_samples.clear()
            for records in self.events.values():records.clear()
        elif self.sim_time is not None and sim==self.sim_time:
            self.duplicate_clock+=1
        self.sim_time=sim
        self.last_clock_wall=wall
        self.clock_samples.append((wall,sim))

    def topic(self,name,wall=None,header_stamp=None):
        if name not in self.events:return
        now=time.monotonic() if wall is None else float(wall)
        if self.sim_time is None:return
        if not math.isfinite(now):return
        stamp=float(header_stamp) if header_stamp is not None else None
        if stamp is not None and (not math.isfinite(stamp) or stamp<=0):
            stamp=None
        self.events[name].append((now,self.sim_time,stamp))

    def snapshot(self,wall=None,mode="synchronized"):
        now=time.monotonic() if wall is None else float(wall)
        clock_active=self.last_clock_wall is not None and now-self.last_clock_wall<=1.5
        # Pausing may keep /clock publications alive with frozen sim stamps.
        recent_clock=[v for v in self.clock_samples if v[0]>=now-4.0]
        rtf=frequency(recent_clock,0)  # not used for RTF; avoid misleading rate.
        if len(recent_clock)>=2:
            wall_delta=recent_clock[-1][0]-recent_clock[0][0]
            sim_delta=recent_clock[-1][1]-recent_clock[0][1]
            rtf=round(max(0.,sim_delta/wall_delta),3) if wall_delta>=.1 else None
        else:rtf=None
        paused=(not clock_active or rtf is not None and rtf<=.005)
        result={}
        for topic,(expected,label) in self.topic_specs.items():
            items=[item for item in self.events[topic] if item[0]>=now-self.window_s]
            latest=items[-1] if items else None
            age=round(now-latest[0],2) if latest else None
            # A single old latched message is not a periodic stream.
            fresh=bool(latest and age<=max(1.5,self.window_s*.6))
            wall_hz=frequency(items,0) if fresh else None
            sim_hz=frequency(items,1) if fresh and clock_active else None
            ages=[item[1]-item[2] for item in items
                  if item[2] is not None]
            header_age=round(ages[-1],4) if ages else None
            backwards=0
            large_gaps=0
            if expected:
                threshold=1.5/expected
                headers=[item[2] for item in items if item[2] is not None]
                for first,second in zip(headers,headers[1:]):
                    if second<first-1e-6:backwards+=1
                    if second-first>threshold:large_gaps+=1
            row={
                "label":label,"expected_sim_hz":expected,
                "wall_hz":wall_hz,"sim_hz":sim_hz,
                "last_message_wall_age_s":age,
                "header_age_sim_s":header_age,
                "header_backward_count":backwards,
                "header_large_gap_count":large_gaps,
                "header_sample_count":len(ages),
                "count_in_window":len(items),
                "rate_basis":"arrival-clock progression",
            }
            row["status"]=assess(row,rtf,paused,mode)
            result[topic]=row
        return {
            "wall_now":time.time(),
            "sim_now":self.sim_time,
            "clock_last_age_s":round(now-self.last_clock_wall,2)
                if self.last_clock_wall is not None else None,
            "rtf":rtf,"paused":paused,
            "clock_jumps":self.jumps,"clock_duplicate_messages":self.duplicate_clock,
            "topics":result,
            "note":(
                "Sim Hz is arrival count per /clock progression; wall Hz is actual "
                "receipt rate. ROS reception is not proof that Gazebo generated every "
                "configured frame. Header gaps suggest dropped/delayed frames but "
                "are not definitive. Joint-state/odom rates do not prove PID loop timing."
            ),
        }
