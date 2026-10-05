# scrobot_mission Regression

## Local shuttle SMC

```bash
ros2 launch scrobot_debug smc_shuttle_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

Final pre-pose controller:

```text
controlled point       base_link
stand-off              1.10 m
position tolerance     0.03 m
yaw tolerance          5 deg
stable time            0.25 s
straight collect       0.30 m/s
overrun                0.10 m

v_R                    0.50 m/s
lambda                 2.50
k_s                    1.20
eta                    0.40
phi                    0.30
k_rho                  0.80
omega max              1.00 rad/s
max angular accel      2.00 rad/s^2
```

Important lesson: `v_R` is the SMC angular-reference speed; actual pre-pose
translation is `v = k_rho * rho * cos(alpha)`. Straight collection is a
separate fixed 0.30 m/s stage.

## Multi-shuttle selection / reacquisition

```bash
ros2 launch scrobot_debug yolo_multi_collect_check.launch.py \
  model_path:=/home/sea/Desktop/yoloshuttle/artifacts/models/gazebo_simple_v2.pt
```

Validated:

```text
multiple raw YOLO detections
 -> mission range filter
 -> first eligible detection frozen in odom
 -> collect
 -> attempted-target exclusion
 -> next eligible detection reacquired
```

The current selector intentionally uses the first eligible filtered detection
in array order. A cost-based selector is an optimization, not a blocker.

## Range gate

Authoritative eligibility:

```text
0.50 <= hypot(x_base, y_base) <= 1.80 m
outside 0.60 m net-pole exclusion
```

Boundary regression is provided by `yolo_range_gate_check.launch.py`.

## Full mission

The only remaining major simulation acceptance test is the integrated run:

```text
initial tag localization
 -> Nav2 four-pass sweep
 -> shuttle interrupt
 -> local collection spree
 -> exact checkpoint return
 -> resume saved path index
 -> fixed-station relocalization
 -> COMPLETE
```

Use `../../../../docs/full_mission_validation.md`.
