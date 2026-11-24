"""Offline regression for single-screen Nav2 scene/planning UX and map QoS."""
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
PAGE=(ROOT/"web"/"index.html").read_text()
SCRIPT=(ROOT/"web"/"app.js").read_text()
RVIZ=yaml.safe_load((ROOT/"config"/"planning.rviz").read_text())


def test_single_page_scene_and_benchmark():
    assert PAGE.count('class="pane"')==0
    assert '<section class="panel" id="scene-panel">' in PAGE
    assert '<section class="panel" id="planning-panel">' in PAGE
    assert 'id="benchmarks"' in PAGE
    assert 'id="courtMap"' in PAGE
    assert 'id="robot-x"' in PAGE and 'id="robot-y"' in PAGE
    assert 'id="robot-yaw"' in PAGE
    assert 'id="box-size"' in PAGE
    assert all('id="'+x+'"' in PAGE for x in
               ('human-ax','human-ay','human-bx','human-by','human-speed'))
    assert 'id="scene-save"' in PAGE and 'id="scene-load"' in PAGE
    assert 'id="map-state"' in PAGE and 'id="costmap-state"' in PAGE
    assert 'id="tf-state"' in PAGE
    assert "/api/scene/save" in SCRIPT
    assert "/api/obstacles/add" in SCRIPT
    assert "/api/scene/place-robot" in SCRIPT


def test_rviz_map_frame_qos_topics():
    manager=RVIZ["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"]=="map"
    displays={d.get("Name"):d for d in manager["Displays"]}
    static=displays["Static poles and tags"]
    live=displays["Live global costmap"]
    assert static["Enabled"] is True and live["Enabled"] is True
    assert static["Topic"]["Value"]=="/map"
    assert static["Topic"]["Durability Policy"]=="Transient Local"
    assert live["Topic"]["Value"]=="/global_costmap/costmap"
    assert live["Topic"]["Durability Policy"]=="Volatile"


def test_map_status_monitor_is_launched():
    source=(ROOT/"launch"/"planning.launch.py").read_text()
    assert "nav2_planner_map_to_odom" in source
    assert "nav2_planner_odom_to_robot" in source
    assert "static_transform_publisher" in source
    assert '"map_status.py"' in source
    assert "map_status_file" in source


def test_planner_tab_drag_clicks_cell_size_and_performance():
    assert '▦ &nbsp; Planner' in PAGE
    assert 'Scene + planning + results' not in PAGE
    assert 'value="human-a"' not in PAGE
    assert 'value="human-b"' not in PAGE
    assert 'value="human"' in PAGE
    assert 'id="resolution" type="number"' in PAGE
    assert 'value="0.05" readonly' in PAGE
    assert 'value="SmacLattice"' in PAGE
    assert 'value="Smac2D"' not in PAGE
    assert 'value="NavFnDijkstra"' not in PAGE
    assert 'id="topic-rate-rows"' in PAGE
    assert "pointerdown" in SCRIPT and "pointermove" in SCRIPT
    assert "setPointerCapture" in SCRIPT
    assert "nextRoutePoint='a'" in SCRIPT
    assert "Math.atan2(dy,dx)" in SCRIPT
    assert "renderRates(status.map_status?.topic_hz)" in SCRIPT


def test_rviz_no_robot_body_thinner_collision_and_fixed_boxes():
    displays={d.get("Name"):d for d in RVIZ["Visualization Manager"]["Displays"]}
    assert not any(d.get("Class")=="rviz_default_plugins/RobotModel"
                   for d in displays.values())
    assert displays["Court markings and outer clearance"]["Topic"]["Value"]=="/nav2_lab/court_lines"
    assert displays["Robot and goal arrows"]["Topic"]["Value"]=="/nav2_lab/pose_arrows"
    assert displays["Camera link only"]["Topic"]["Value"]=="/nav2_lab/camera_link"
    assert displays["Static poles and tags"]["Alpha"]<=.35
    assert displays["Live global costmap"]["Alpha"]<=.25
    assert displays["Live global costmap"]["Draw Behind"] is True
    overlay=(ROOT/"map_status.py").read_text()
    assert "def read_footprint(" in overlay
    assert "perimeter.scale.x=.025" in overlay
    assert "Marker.CUBE,frame=\"map\"" in overlay
    assert "self.court_pub.publish(self.fixed_court)" in overlay
    assert "self.court_published=True" in overlay
    assert "Marker.LINE_LIST" not in overlay
    launch=(ROOT/"launch"/"planning.launch.py").read_text()
    assert '"robot_visual_mode": "simple"' in launch
    assert '"--nav2-params", v["nav2_params_file"]' in launch


def test_lite_launch_avoids_unneeded_pipeline_nodes():
    source=(ROOT/"launch"/"planning.launch.py").read_text()
    assert '"control_stack.launch.py"' not in source
    assert '"localization.launch.py"' not in source
    assert '"perception.launch.py"' not in source
    assert '"simulation.launch.py"' not in source
    assert '"spawn_robot.launch.py"' in source
    assert '"pointcloud_to_laserscan_node"' in source
    assert '"depth_scan_self_filter"' in source
    bridge=(ROOT/"config"/"depth_bridge.yaml").read_text()
    assert "/camera/camera/depth/points" in bridge
    assert "/clock" in bridge
    assert "/camera/camera/color" not in bridge


def test_court_line_geometry_from_real_sdf():
    import sys
    sys.path.insert(0,str(ROOT))
    from court_visual import court_lines
    world=ROOT.parents[2]/"src/scrobot_simulation/worlds/badminton_court.sdf"
    geometry=court_lines(world)
    assert geometry["court_length_m"] == 15.4
    assert geometry["court_width_m"] == 7.2
    assert geometry["venue_length_m"] == 21.0
    assert geometry["venue_width_m"] == 12.0
    assert len(geometry["court_outline"]) == 4
    assert len(geometry["clearance_outline"]) == 4
    assert len(geometry["court_lines"]) >= 6


def test_log_color_escapes_are_removed():
    import sys
    sys.path.insert(0,str(ROOT.parent))
    from planning.web_server import sanitize_log
    colored="\x1b[32m[INFO]\x1b[0m ready\r\n\x1b]0;title\x07plain"
    cleaned=sanitize_log(colored)
    assert cleaned=="[INFO] ready\nplain"
    assert "\x1b" not in cleaned


def test_moving_cylinder_uses_explicit_add_button():
    handler=SCRIPT.split("$('courtMap').addEventListener('pointerdown'")[1]
    handler=handler.split("$('courtMap').addEventListener('pointermove'")[0]
    assert "placed(p[0],p[1],'human-'+endpoint)" in handler
    assert "nextRoutePoint=endpoint==='a'?'b':'a'" in handler
    assert "/api/obstacles/add" not in handler
    assert "$('add-human').onclick=" in SCRIPT
    assert 'id="add-human">Add cylinder</button>' in PAGE


def test_live_planning_all_plugins_route_and_controls():
    runner=(ROOT/"runner.py").read_text()
    service=(ROOT/"web_server.py").read_text()
    assert "def run_live(" in runner
    assert "names=list(PLANNERS)" in runner
    assert "while not STOP:" in runner
    assert "--live-interval-s" in runner
    assert '"mode":"live"' in runner
    assert '"/api/plan":lambda x:self.service.trial_start(x)' in service
    assert 'self.trial_mode!="live"' in service
    assert 'id="planning-mode"' in PAGE
    assert 'id="live-interval"' in PAGE
    assert "renderResults(status.summary)" in SCRIPT
    assert "summary?.mode==='live'" in SCRIPT



def test_court_painted_lines_reuse_fixed_sdf_box_dimensions():
    import sys
    sys.path.insert(0,str(ROOT))
    from court_visual import court_lines,box_segments
    world=ROOT.parents[2]/"src/scrobot_simulation/worlds/badminton_court.sdf"
    geom=court_lines(world)
    assert len(geom["court_line_boxes"])>=6
    assert len(geom["court_outline_boxes"])==4
    assert len(geom["clearance_outline_boxes"])==4
    assert any(item["width"]==.04 for item in geom["court_line_boxes"])
    assert all(item["length"]>0 for item in geom["court_line_boxes"])
    assert len(box_segments(15.4,7.2,.05))==4
