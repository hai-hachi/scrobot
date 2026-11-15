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
    assert "nav2_lab_map_to_odom" in source
    assert "static_transform_publisher" in source
    assert '"map_status.py"' in source
    assert "map_status_file" in source
