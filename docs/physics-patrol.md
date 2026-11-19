# MuJoCo workshop inspection

A synthetic workshop with machinery, marked stations and a crossing service cart. The main view follows the simulated Go2. The side panels show the planned route, the integrated body trace and the camera attached to the chassis.

[Watch the full 1600 × 900 recording](../assets/go2-physics-patrol.mp4?raw=1) · [Open the full-size still](../assets/go2-physics-patrol.jpg) · [Inspect the run record](data/physics-patrol-mission.json)

## What ran

1. The original project’s `inspection_planner.py` supplies `GridPath`. The wrapper passes in a synthetic occupancy grid with 0.15 m cells and a 0.48 m planning margin. `GridPath.solve()` and `GridPath.reconstruct()` call Dijkstra on the eight-neighbor graph and reconstruct each leg.
2. At 4.0 s a scripted cart crossing updates the occupied cells. The same planner computes a detour from the simulated pose. A waypoint follower uses simulator pose feedback. The cart is a scripted object and the occupancy update is an event, with no LiDAR detector or SLAM in this scenario.
3. The imported [rl_sar Go2 policy](https://github.com/fan-ziqi/rl_sar/tree/376d42c9b128f963ab08579762d5a216a976ce39/policy/go2/robot_lab) runs at 50 Hz from joint state, angular velocity and projected gravity. The demonstration adds velocity feedback and joint PD torque control. MuJoCo integrates at 200 Hz and determines how the free-floating Go2 body moves through foot contacts. The renderer reads the saved simulation states.
4. At each station the controller switches to a standing posture. Capture requires 0.40 s of consecutive samples meeting the position, heading, motion and target visibility checks. The renderer saves an RGB frame from the exact recorded capture pose. The camera is a chassis-attached pinhole model with a 56° vertical field of view.

## Recorded outcome

This configuration executed 16,104 physics steps over 80.52 simulated seconds and produced two captures. No robot-to-cart or robot-to-fixture contacts were recorded. The same run recorded 674 non-foot floor contact samples, including 78 from the base, and reached 39.6° of body tilt. These are sampled contact observations, not counts of separate falls. Other controller configurations encountered obstacle contacts, excessive tilt or failed to complete both stops. This is a demonstration of integrated motion, planning and image capture under the configured conditions, with visible gait instability.

| Station | Capture time | Position error | Body heading error to target | Planar speed | RGB frame |
| --- | ---: | ---: | ---: | ---: | --- |
| 1 | 35.92 s | 0.244 m | 4.65° | 0.001 m/s | [Station 1](../assets/simulated-capture-1.png) |
| 2 | 78.50 s | 0.271 m | 12.85° | 0.003 m/s | [Station 2](../assets/simulated-capture-2.png) |

The position error is measured from the synthetic station center. The heading error is the chassis yaw relative to the marker direction. The record ends after the second capture and stand. Field localization, camera calibration and deployment performance are separate validation tasks.

## Run with the original project source

Use Python 3.14 for the recorded package versions, the [Menagerie Go2 model](https://github.com/google-deepmind/mujoco_menagerie/tree/370c43965323c0944b433b202aac5058b5f907a9/unitree_go2) and the [pinned efferent ONNX conversion](https://github.com/Eximius-Labs/efferent/blob/f51e1af953afe1016dae9ceeff2b5328fa51781d/examples/go2_walk/policy.onnx). Keep the model’s `assets/` beside `go2.xml`. The original inspection planner is supplied from your existing project checkout. The exact model XML, ONNX and field entry hashes are in the run record.

```bash
python -m pip install mujoco==3.10.0 numpy==2.5.5 scipy==1.18.1 onnxruntime==1.32.0 Pillow==13.0.0
python -m examples.physics_patrol \
  --robot-xml /path/to/unitree_go2/go2.xml \
  --policy /path/to/policy.onnx \
  --field-planner /path/to/inspection_planner.py \
  --output patrol-run
MUJOCO_GL=egl python -m examples.render_patrol patrol-run \
  /path/to/unitree_go2/go2.xml "$(command -v ffmpeg)"
```

The simulation writes the route, capture diagnostics and state snapshots. Rendering requires EGL, DejaVu Sans and FFmpeg with H.264 encoding.
