"""Clock-aware ROS topic metrics with accurate timestamp-gap accounting.

Pure Python. All rates are observations at a ROS subscriber, not a claim about
Gazebo native frames or PID callbacks. Message timestamps (including serialized
camera headers) provide jitter/skip evidence separate from ROS arrival jitter.
"""
from collections import deque
import math
import struct
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
    "/global_costmap/costmap":(None,"Nav2 full costmap (latched)"),
    "/global_costmap/costmap_updates":(None,"Nav2 incremental costmap updates"),
    "/cmd_vel_nav":(None,"Nav2 output; wall-scheduled controller"),
    "/cmd_vel_approach":(None,"Local collection SMC output; active only"),
    "/cmd_vel_auto":(None,"Autonomous mux output"),
    "/cmd_vel_smoothed":(None,"Velocity smoother output"),
    "/cmd_vel_motor_target":(None,"Motor response input; follows velocity smoother"),
    "/diff_drive_controller/cmd_vel":(100.0,"Simulation motor-response output"),
    "/perception/shuttle_detections":(None,"YOLO detections if published"),
}
NONPERIODIC_TOPICS={
    "/global_costmap/costmap",
    "/global_costmap/costmap_updates",
}


def stamp_seconds(stamp):
    if stamp is None:return None
    sec=float(stamp.sec)+float(stamp.nanosec)*1e-9
    return sec if math.isfinite(sec) and sec>0 else None


def serialized_header_stamp(raw):
    """Extract the first ROS std_msgs/Header stamp from CDR without decoding payload.

    Image, PointCloud2, OccupancyGrid, OccupancyGridUpdate and TwistStamped
    have Header as their first field. CDR starts with a 4-byte encapsulation
    marker. After that are int32 sec and uint32 nanosec. Do NOT use this for
    messages whose first member is not Header, or for unsupported CDR encodings.
    """
    try:
        if not isinstance(raw,(bytes,bytearray,memoryview)) or len(raw)<12:
            return None
        if raw[0]!=0 or raw[1] not in (0,1):
            return None
        endian="<" if raw[1]==1 else ">"
        sec,nanosec=struct.unpack_from(endian+"iI",raw,4)
        if sec<0 or nanosec>=1_000_000_000:
            return None
        result=sec+nanosec*1e-9
        return result if result>0 else None
    except (ValueError,TypeError,struct.error,IndexError):
        return None


def frequency(items,index):
    if len(items)<2:return None
    delta=items[-1][index]-items[0][index]
    return round((len(items)-1)/delta,2) if delta>0 else None


def percentile(values,q):
    if not values:return None
    vals=sorted(v for v in values if math.isfinite(v))
    if not vals:return None
    pos=(len(vals)-1)*q
    a=int(pos)
    fraction=pos-a
    return vals[a]*(1-fraction)+vals[min(a+1,len(vals)-1)]*fraction


def interval_quality(items,expected):
    """Timing distribution from individual header stamps, not averaged Hz."""
    stamps=[x[2] for x in items if x[2] is not None]
    gaps=[b-a for a,b in zip(stamps,stamps[1:])]
    positive=[gap for gap in gaps if gap>0]
    nominal=1./expected if expected else None
    jitter=[abs(g-nominal) for g in positive] if nominal else []
    return {
        "header_interval_samples":len(positive),
        "header_interval_p50_ms":round(percentile(positive,.5)*1000,3)
            if positive else None,
        "header_interval_p95_ms":round(percentile(positive,.95)*1000,3)
            if positive else None,
        "header_interval_max_ms":round(max(positive)*1000,3)
            if positive else None,
        "header_abs_jitter_p95_ms":round(percentile(jitter,.95)*1000,3)
            if jitter else None,
    }


def assess(row,rtf,paused,mode="synchronized"):
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
        self.total_counts={topic:0 for topic in self.topic_specs}
        self.total_header_gaps={topic:0 for topic in self.topic_specs}
        self.total_header_backward={topic:0 for topic in self.topic_specs}
        self.previous_stamps={}
        self.first_seen_sim={}
        self.last_seen_sim={}
        self.last_seen_wall={}
        self.sim_time=None
        self.last_clock_wall=None
        self.jumps=0
        self.duplicate_clock=0

    def add_topic(self,name,expected=None,label=None):
        if name in self.events:return
        self.topic_specs[name]=(expected,label or name)
        self.events[name]=deque(maxlen=3200)
        self.total_counts[name]=0
        self.total_header_gaps[name]=0
        self.total_header_backward[name]=0

    def clock(self,sim,wall=None):
        wall=time.monotonic() if wall is None else float(wall)
        sim=float(sim)
        if not math.isfinite(sim) or not math.isfinite(wall):return
        if self.sim_time is not None and sim<self.sim_time-1e-6:
            self.jumps+=1
            self.clock_samples.clear()
            for records in self.events.values():records.clear()
            self.previous_stamps.clear()
            self.total_counts=dict.fromkeys(self.events,0)
            self.total_header_gaps=dict.fromkeys(self.events,0)
            self.total_header_backward=dict.fromkeys(self.events,0)
            self.first_seen_sim.clear()
            self.last_seen_sim.clear()
            self.last_seen_wall.clear()
        elif self.sim_time is not None and sim==self.sim_time:
            self.duplicate_clock+=1
        self.sim_time=sim
        self.last_clock_wall=wall
        self.clock_samples.append((wall,sim))

    def topic(self,name,wall=None,header_stamp=None):
        if name not in self.events:return
        now=time.monotonic() if wall is None else float(wall)
        if self.sim_time is None or not math.isfinite(now):return
        stamp=float(header_stamp) if header_stamp is not None else None
        if stamp is not None and (not math.isfinite(stamp) or stamp<=0):
            stamp=None
        self.events[name].append((now,self.sim_time,stamp))
        self.total_counts[name]+=1
        self.first_seen_sim.setdefault(name,self.sim_time)
        self.last_seen_sim[name]=self.sim_time
        self.last_seen_wall[name]=now
        if stamp is not None:
            old=self.previous_stamps.get(name)
            expected=self.topic_specs[name][0]
            if old is not None:
                gap=stamp-old
                if gap<-1e-6:self.total_header_backward[name]+=1
                if expected and gap>1.5/expected:
                    self.total_header_gaps[name]+=1
            self.previous_stamps[name]=stamp

    def snapshot(self,wall=None,mode="synchronized"):
        now=time.monotonic() if wall is None else float(wall)
        clock_active=self.last_clock_wall is not None and now-self.last_clock_wall<=1.5
        recent_clock=[v for v in self.clock_samples if v[0]>=now-4.]
        rtf=None
        if len(recent_clock)>=2:
            dw=recent_clock[-1][0]-recent_clock[0][0]
            ds=recent_clock[-1][1]-recent_clock[0][1]
            if dw>=.1:rtf=round(max(0.,ds/dw),3)
        paused=(not clock_active or rtf is not None and rtf<=.005)
        result={}
        for name,(expected,label) in self.topic_specs.items():
            items=[x for x in self.events[name] if x[0]>=now-self.window_s]
            latest_wall=self.last_seen_wall.get(name)
            last_age=round(now-latest_wall,2) if latest_wall is not None else None
            recent=bool(items and last_age<=max(1.5,self.window_s*.6))
            wall_hz=frequency(items,0) if recent else None
            sim_hz=frequency(items,1) if recent and clock_active else None
            offsets=[x[1]-x[2] for x in items if x[2] is not None]
            stamps=[x[2] for x in items if x[2] is not None]
            backwards=sum(b<a-1e-6 for a,b in zip(stamps,stamps[1:]))
            gaps=sum(b-a>1.5/expected for a,b in zip(stamps,stamps[1:])) if expected else 0
            row={
                "label":label,"expected_sim_hz":expected,
                "wall_hz":wall_hz,"sim_hz":sim_hz,
                "last_message_wall_age_s":last_age,
                "header_clock_offset_sim_s":round(offsets[-1],4) if offsets else None,
                "header_backward_count":backwards,
                "header_large_gap_count":gaps,
                "header_large_gap_total":self.total_header_gaps[name],
                "header_backward_total":self.total_header_backward[name],
                "header_sample_count":len(stamps),
                "count_in_window":len(items),
                "count_total":self.total_counts[name],
                "first_seen_sim_s":self.first_seen_sim.get(name),
                "last_seen_sim_s":self.last_seen_sim.get(name),
                "rate_basis":"arrival-clock progression",
            }
            row.update(interval_quality(items,expected))
            if name in NONPERIODIC_TOPICS:
                row["status"]="observed" if row["count_total"] else "unknown"
            else:row["status"]=assess(row,rtf,paused,mode)
            result[name]=row
        return {
            "wall_now":time.time(),"sim_now":self.sim_time,
            "clock_last_age_s":round(now-self.last_clock_wall,2)
                if self.last_clock_wall is not None else None,
            "rtf":rtf,"paused":paused,
            "clock_jumps":self.jumps,
            "clock_duplicate_messages":self.duplicate_clock,
            "topics":result,
            "note":(
                "Per-topic Hz is based on ROS subscriber reception and the latest "
                "clock. Header interval statistics use source timestamps, not ROS "
                "arrival order. Header gaps are not proof of dropped sensor frames. "
                "Total gap counters count each observed gap once since monitor reset, "
                "while window counts overlap. Latched costmaps are NOT rate targets. "
                "Command topic frequency is a controller OUTPUT proxy, not a PID "
                "callback or proof of Nav2 controller scheduler frequency."
            ),
        }
