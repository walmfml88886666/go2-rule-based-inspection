# Physical simulation on project SLAM maps

The indoor and outdoor scenes come from the project's archived LiDAR mapping and inspection work. Each run uses the archived terrain surface, obstacle primitives, waypoint sequence and observation poses. The planning output is the original project route, whose graph-search stage uses Dijkstra. This experiment tests physical execution of that stored output.

[Indoor video](../assets/slam-indoor.mp4?raw=1) · [Outdoor video](../assets/slam-outdoor.mp4?raw=1) · [Aggregate run evidence](data/slam-physics-results.json)

## Source data

The local source is the 2026-10-30 three-profile comparison archive and its frozen-input snapshot. The selected files were extracted to a separate working folder and checked against the archived SHA-256 mapping.

| Input | Indoor | Outdoor |
| --- | --- | --- |
| LiDAR localization cloud | 246,739 points | 1,385,388 points |
| Original inspection task | 22 targets and a return leg | 8 targets and a return leg |
| Static scene geometry | 16,998 primitives including terrain | 61,417 primitives including terrain |
| Terrain representation | Archived 0.20 m height field | Archived 0.20 m height field |
| Collision scene | Archived occupancy-0.50 derivative | Archived occupancy-0.50 derivative |

The collision scene is the archive's occupancy-0.50 sensitivity model. It carries the map residuals and geometric approximations of that model. Every source terrain and obstacle position and box dimension is checked when the model is compiled. The view uses translucent voxels for visibility. The right-hand map panel uses the archived point-cloud appearance data and plots the actual simulated trace beside the stored plan.

The aggregate record contains source checksums and diagnostics. Original point clouds, task bundles and state arrays remain with the project archive. Public readers can verify media against `docs/asset-provenance.json` and `examples/go2_controller.py` against `publication_check.published_controller_sha256`. The recorded `controller_sha256` identifies the executed, earlier-formatted adapter; its syntax tree was checked against the published adapter. Checking the twelve source files and full state archives requires access to the original project data; published hashes alone do not provide independent reproduction.

## Motion and capture protocol

MuJoCo 3.10.0 integrates the floating-base Go2 and its joint torques at 500 Hz. The archived [rl_sar robot_lab policy conventions](https://github.com/fan-ziqi/rl_sar/tree/376d42c9b128f963ab08579762d5a216a976ce39/policy/go2/robot_lab) define joint ordering, observation scaling, action scaling and PD gains. The same archived ONNX conversion is used. The simulation includes the project's experimental ideal-velocity feedback adapter. The waypoint follower reads simulator pose. Initial XY and yaw come from the task start pose. Initial height is the archived terrain height plus 0.32 m for settling, and initial joints use the archived policy's default angles.

The angular input is taken from the floating joint in chassis axes. A regression test verifies the axes against a model with rotated inertia, guarding the error that destabilized the previous workshop example.

At each original observation pose, a capture request requires 0.60 s of consecutive stable samples with planar speed below 0.01 m/s, yaw rate below 0.06 rad/s and heading error within 5°. The follower uses the original stop order and keeps the task's three-second photo dwell. Any obstacle contact or non-foot floor contact stops the run. These are explicit simulation checks.

## Recorded outcomes

| Scene | Inspection poses reached | Return | Simulated duration | Travel | Maximum body tilt | Obstacle contact samples | Non-foot floor contact samples |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| Indoor | 4/22 | Incomplete | 193.26 s | 47.43 m | 5.49° | 1 | 0 |
| Outdoor | 8/8 | Completed | 635.56 s | 227.76 m | 7.62° | 0 | 0 |

The indoor run stopped on the segment following MHQ-004. This contact is an outcome of the archived occupancy-0.50 collision model, stored route and current follower together; this run does not isolate map discretization, planned clearance or tracking error as the cause. The outdoor run reached all eight inspection poses and the stored return pose. Capture requests record position and heading errors for each stop. The videos retain the indoor stopping outcome.

These are conditional simulation runs in a coarse map-derived model. A repeated deterministic run checks reproducibility. The earlier [three-profile comparison](navigation-comparison.md) uses its own archived execution conditions and results.

[Indoor capture request M-001](../assets/slam-indoor-capture.png) · [Outdoor capture request D-001](../assets/slam-outdoor-capture.png)

## Camera view

The camera panel is a geometric pinhole proxy attached to the Go2 chassis. It renders the scan-derived surface. It uses a 56° vertical field of view, consistent with the project's rounded archived field-of-view value. The archived camera check supplies intrinsic calibration but no synchronized camera-to-body optical transform. The proxy's relative translation is explicitly modeled as (0.257, 0, 0.047) m and its optical axis follows chassis +X. Capture requests assess stopped pose and heading, with image quality and target recognition left for calibrated camera validation.

## Reproduce with project inputs

Prepare the indexed input folder using the frozen-input mapping in the original project archive. Keep the source receipt as `verified-inputs.json`. The CLI checks the receipt against the published frozen-source checksums, then verifies each input file before use. The owner-side export uses the original indexed archive names: indoor input indices 002, 081, 109, 110, 113, 120, and outdoor indices 005, 082, 111, 112, 115, 121. Their member names and checksums are in the aggregate record.

```bash
python -m pip install mujoco==3.10.0 numpy==2.5.5 scipy==1.18.1 onnxruntime==1.32.0 Pillow==13.0.0
python -m examples.slam_patrol \
  --inputs /path/to/verified-inputs --scene indoor \
  --robot-xml /path/to/unitree_mujoco/go2.xml \
  --policy /path/to/archived/policy.onnx --output indoor-run
MUJOCO_GL=egl python -m examples.render_slam \
  --inputs /path/to/verified-inputs --scene indoor \
  --robot-xml /path/to/unitree_mujoco/go2.xml \
  --run indoor-run --ffmpeg /path/to/ffmpeg
```

Use `--scene outdoor` for the second map. The simulation writes the full state trace and event record. Rendering exports selected excerpts at 1x simulated speed with visible timestamps. It requires EGL, DejaVu Sans and an H.264-capable FFmpeg.
