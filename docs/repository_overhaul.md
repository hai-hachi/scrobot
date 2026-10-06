# Repository Overhaul and Branch Disposition

Repository consolidation performed after the validated 50/50 full-mission run.

The integration source was `perception-yolo-v2`. A clean consolidation branch,
`repo-overhaul-20261006`, was created from that validated head before touching
`main`.

## Retained in the consolidated tree

- final four-pass sweep mission and checkpoint return/resume;
- production YOLO + aligned-depth perception;
- final local SMC controller including close-target staging clamp;
- world-level physical shuttle removal;
- local-odometry and collection-session evaluators;
- master RViz and final mission debug launch;
- current AprilTag localization and recovery flow;
- Jetson Orin Docker/deployment support recovered from hardware branches;
- STM32 protocol-v2 ros2_control hardware interface at 1 Mbaud;
- D435i physical profiles, SensorDataQoS IMU transformer and lightweight
  depth-scan fallback.

Obsolete external HMC5883L support was intentionally not restored.

## Branch audit

### Fully superseded / no unique commits relative to their successor

```text
first-seen-shuttle-policy
integrate-shuttle-simulation
local-visual-collection-v1
new-strategy-v1
patrol-v1-archive
patrol-v1-collision-recovery
patrol-v1-eval-refactor
patrol-v1-initial-local-only
qos-latency-cleanup
shuttle-perception-sim-v1
simulation-systematic-test2
sweep-local-collect-v1
tag-approach-v3-selection-lock
yolo-simple-capture
```

These branches are historical development checkpoints. Their useful behavior is
already present in the consolidated stack.

### Diverged experiments intentionally not merged wholesale

| Branch | Unique work | Disposition |
| --- | ---: | --- |
| `depth_pointcloud` | 1 commit | superseded by current depth pipeline |
| `midway-patrol-fix` | 1 commit | stale patrol/back-up-file experiment |
| `optimized-nav2-mow-v1` | 14 commits | superseded by final four-pass mission |
| `global-sweep-route-v1` | 42 commits | superseded by final sweep/local-collect architecture |
| `shuttle-sim-v1` | 3 commits | superseded by current shuttle model/manager |
| `shuttle-simulation-v1` | 33 commits | superseded by current spawn/removal systems |
| `simulation-systematic-test-c98fee65` | 67 commits | debug/control refactor superseded by current evaluation/debug package |

### Physical hardware branches

| Branch | Disposition |
| --- | --- |
| `rpi-real-hardware-bringup` | older Pi-oriented hardware path; current useful protocol/driver work superseded by Orin branch |
| `orin-docker-bringup` | useful Orin/STM32/D435i support selectively migrated and normalized to current production interfaces |

The old hardware branches were not merged directly because they also changed
mission/control/navigation defaults from an earlier software generation. Only
hardware-specific capabilities were restored so the validated simulation
behavior is not regressed.

### Main divergence

Before consolidation, `main` contained 50 commits not in the final integration
branch, while `perception-yolo-v2` contained 722 commits not in `main`.
The main-only commits were reviewed by behavior. Their useful changes—D435i
geometry, D435i-only localization, tag recovery, Nav2 footprint, mission limits,
SMC handoff and documentation—are represented by newer or independently
implemented versions in the consolidated final tree.

## Consolidation result

The audited tree was merged to `main` on 2026-10-06 with merge commit:

```text
d94d204b8c99a91f09b3b63741155ed20f347b00
Merge validated repository overhaul into main
```

The merge has two parents: the previous `main` head and the audited
`repo-overhaul-20261006` head. The resulting tree is exactly the audited
consolidation tree, so both histories are retained without reintroducing stale
files from the older main line.

The previous production head is additionally preserved at:

```text
archive/main-pre-overhaul-20261006
```

GitHub pull request #4 records the consolidation review and merge.

## Final branch policy

After merge, `main` is the production branch.

Historical experiment branches should be treated as read-only archive material
and may be deleted once no external deployment still references them. New work
should branch from `main`, not from the archived patrol/simulation branches.
