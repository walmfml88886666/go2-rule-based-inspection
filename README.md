# Go2 Patrol

Rule-based LiDAR SLAM inspection with Unitree Go2 EDU. The system plans routes with Dijkstra and links each field run to its trajectory and inspection photos.

![ROS 2 Humble field stack](https://img.shields.io/badge/Field%20stack-ROS%202%20Humble-334b80)
![Python 3.11 audit](https://img.shields.io/badge/Route%20audit-Python%203.11-2776aa)
![MuJoCo 3.10.0](https://img.shields.io/badge/Simulation-MuJoCo%203.10.0-087e72)
[![Repository rights and attribution](https://img.shields.io/badge/License-Rights%20notice-9a6240)](LICENSE)

## Overview

[![Indoor and outdoor height-colored LiDAR maps with planned routes, observation poses and the inspection workflow](assets/slam-overview.jpg)](docs/system.md)

## XT16 mapping calibration

A matched 1,040-second indoor replay used the same LIO-SAM code and sensor recording. Updating the IMU axis rotation and roll/pitch fusion weight together reduced the estimated trajectory's **vertical span from 5.97 m to 0.89 m**. Both side views share the same axes. [Calibration method and evidence](docs/mapping.md)

![Height-colored side views of the indoor mapping calibration comparison with estimated trajectories](assets/mapping-calibration-ab.jpg)

The XT16 frame adaptation rotates Go2 IMU vectors into LiDAR axes. The archived A/B changes the roll/pitch fusion weight from 0.01 to 0.15 alongside that rotation.

$$
mathbf v_{mathrm{LiDAR}}=
egin{bmatrix}0&1&0\-1&0&0\0&0&1end{bmatrix}mathbf v_{mathrm{IMU}},
qquad
Delta z=max_i z_i-min_i z_i.
$$

## Navigation and line-of-sight layers

The planner estimates local ground from the point cloud and the nearest recorded mapping pose. Near the trajectory, a height ceiling filters canopy and overhead returns before the ground percentile is computed. Here $g_k=z_k-0.485$ m and $d_c$ is the horizontal distance from cell $c$ to that pose.

$$
z_{max}(c)=g_k+d_c	an(25^circ)+0.16+0.05,qquad
hat g_c=Q_{0.15}igl({z_i:z_ile z_{max}(c),,iin c}igr).
$$

In each 0.15 m cell, the two masks count returns in different bands above $hat g_c$. The navigation layer checks body clearance. The line-of-sight layer screens candidate viewing poses for higher obstructions.

$$
N(c)=mathbf 1[n_c(0.16,,0.6296)ge 2],
qquad
V(c)=mathbf 1[n_c(0.90,,2.20)ge 2],
qquad
n_c(a,b)=#{iin c:a<z_i-hat g_c<b}.
$$

## Datasets

Download the [indoor and outdoor datasets](datasets/). Each includes a metric XYZ point cloud, planned route and ordered observation poses in a shared local frame.

## Field demos

| Indoor inspection | Outdoor gravel crossing |
| :---: | :---: |
| [![Go2 walking to an indoor inspection target](assets/field-exterior-indoor.webp)](assets/field-exterior-indoor.mp4?raw=1) | [![Go2 walking from pavement onto gravel](assets/field-exterior-outdoor.webp)](assets/field-exterior-outdoor.mp4?raw=1) |

### Indoor mission replay · 22/22 captures

[![Full recorded indoor trajectory and capture timeline through 22 of 22](assets/indoor-mission-replay.webp)](assets/indoor-mission-replay.mp4?raw=1)

## SLAM-map physical simulation

The Go2 follows the original inspection routes on LiDAR-derived terrain in MuJoCo. The fixed maps show Go2 heading, driven path, active route and the next observation pose. The insets show the same moments at 1× speed.

**Outdoor · 8/8 inspection stops and return**

[![Close view of Go2 walking on the outdoor SLAM map](assets/slam-outdoor-route.webp)](assets/slam-outdoor-route.mp4?raw=1)

**Indoor · 4/22 inspection stops before a contact-triggered safety stop**

[![Close view of Go2 walking on the indoor SLAM map](assets/slam-indoor-route.webp)](assets/slam-indoor-route.mp4?raw=1)

## Navigation comparison

[![Recorded indoor and outdoor comparison of the project stack, NavFn A* and NavFn Dijkstra](assets/archived-comparison.webp)](docs/navigation-comparison.md)

## Run the route audit

```bash
python -m pip install -r requirements.txt
python -m examples.offline_audit
```
