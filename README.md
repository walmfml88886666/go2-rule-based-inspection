<p align="center"><img src="assets/banner.svg" alt="Go2 Patrol — rule-based indoor and outdoor inspection" width="100%"></p>

# Go2 Patrol

**Rule-based indoor and outdoor inspection with the Unitree Go2 EDU.** The field system turns LiDAR maps and annotated targets into inspection routes, then ties robot trajectories, images, and logs back to each mission.

**[Field demos](#field-demos)** · **[SLAM-map simulation](#slam-map-physical-simulation)** · **[Architecture](#system-architecture)** · **[Quick start](#quick-start)** · **[Downloads](https://github.com/walmfml88886666/go2-rule-based-inspection/releases/latest)** · **[Discussions](https://github.com/walmfml88886666/go2-rule-based-inspection/discussions)**

## Field demos

### Indoor mission replay

[![Indoor mission replay showing the recorded route on a LiDAR map with photos from the same inspection](assets/indoor-mission-replay.webp)](https://github.com/walmfml88886666/go2-rule-based-inspection/releases/download/v0.3.0/indoor-mission-replay.mp4)

A recorded indoor mission links the **measured trajectory** to **22 logged photo captures**. The time-compressed replay plots the route over the LiDAR map and shows eight photos at their recorded capture events.

**[Full 1600 × 900 MP4](https://github.com/walmfml88886666/go2-rule-based-inspection/releases/download/v0.3.0/indoor-mission-replay.mp4)** · [High-resolution still](assets/indoor-mission-poster.jpg) · [Field media notes](docs/gallery.md)

### First-person field recordings

These previews open 720p excerpts from three separate field runs.

| **Indoor patrol** | **Outdoor patrol · Day** | **Outdoor patrol · Night** |
| :---: | :---: | :---: |
| [![Indoor first-person field recording](assets/indoor-field.webp)](https://github.com/walmfml88886666/go2-rule-based-inspection/releases/download/v0.2.0/indoor-field.mp4) | [![Outdoor daylight first-person field recording](assets/outdoor-day-field.webp)](https://github.com/walmfml88886666/go2-rule-based-inspection/releases/download/v0.2.0/outdoor-day-field.mp4) | [![Outdoor night first-person field recording](assets/outdoor-night-field.webp)](https://github.com/walmfml88886666/go2-rule-based-inspection/releases/download/v0.2.0/outdoor-night-field.mp4) |

### From point cloud to inspection route

The images below project historical LiDAR corner points, planned routes, and target-to-observation links onto the map plane. The colored paths show the routes planned before each mission. Open either map to inspect the 1600 × 1000 original.

| **Indoor · 22 planned stops** | **Outdoor · 8 planned stops** |
| :---: | :---: |
| [![Indoor point cloud and planned route](assets/indoor-field-map.png)](assets/indoor-field-map.png) | [![Outdoor point cloud and planned route](assets/outdoor-field-map.png)](assets/outdoor-field-map.png) |

## SLAM-map physical simulation

The Go2 follows the original inspection route in environments derived from the project's recorded LiDAR SLAM maps. The terrain, obstacle voxels and inspection poses are read from verified project archives.

**Outdoor · all 8 inspection poses and return**

[![Go2 completing the recorded outdoor inspection route](assets/slam-outdoor.webp)](assets/slam-outdoor.mp4?raw=1)

**Indoor · 4/22 inspection poses, then safety stop**

[![Go2 following the recorded indoor map and inspection route](assets/slam-indoor.webp)](assets/slam-indoor.mp4?raw=1)

[Download indoor MP4](assets/slam-indoor.mp4?raw=1) · [Download outdoor MP4](assets/slam-outdoor.mp4?raw=1) · [Input provenance, outcomes and capture errors](docs/physics-patrol.md)

| Recorded map | Inspection poses reached | Return | Simulated time / travel | Ending |
| --- | ---: | --- | --- | --- |
| Indoor | 4/22 | Incomplete | 193.26 s / 47.43 m | After MHQ-004: safety stop after one obstacle contact sample |
| Outdoor | 8/8 | Completed | 635.56 s / 227.76 m | Returned to the original start |

The timestamped excerpts show integrated joint torque and foot-contact motion. The camera panel uses an approximate virtual camera on scan geometry, without measured camera-to-body extrinsics or RGB textures. It does not validate real camera images or target recognition. Historical field imagery appears in the field demos above.

## System architecture

<p align="center">
  <img src="assets/robot.jpg" alt="The project’s Unitree Go2 EDU fitted with a Hesai XT16 LiDAR" width="620"><br>
  <sub>Field platform · Go2 EDU with Hesai XT16 LiDAR</sub>
</p>

[![Hardware and software architecture](assets/architecture.svg)](assets/architecture.svg)

1. **Map the site.** Record LiDAR and IMU data and build a point cloud with the project's ROS 2 / LIO-SAM workflow.
2. **Annotate inspection targets.** Record each target's position, category, and identifier on the map.
3. **Plan a mission.** Check traversability, elevation, robot footprint, camera line of sight, and visit order before exporting waypoints and their matching map layers.
4. **Run the inspection.** Sync the mission bundle, run onboard preflight checks, then navigate to observation poses and capture images.
5. **Review the evidence.** Associate the trajectory, target images, and event logs with the original map and mission.

**[System notes](docs/system.md)** · **[Editable draw.io diagram](docs/architecture.drawio)** · **[Field media and data provenance](docs/gallery.md)**

## Quick start

Run the public route audit and tests with Python 3.11 or a compatible Python 3 version.

```bash
git clone https://github.com/walmfml88886666/go2-rule-based-inspection.git
cd go2-rule-based-inspection
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m examples.offline_audit
python -m unittest discover -s tests -v
```

The [Linux x86-64 package](https://github.com/walmfml88886666/go2-rule-based-inspection/pkgs/container/go2-rule-based-inspection) also runs the offline audit. The old A* grid examples are documented as [archived developer fixtures](docs/simulation.md).

The [route geometry module](tools/route_intersections.py) detects crossings and measures collinear retracing. The [visit-order audit](tools/visit_order_metrics.py) enumerates feasible orderings for small sets of fixed observation stops. The full field GUI, ROS workspace, vehicle configuration, deployment software, and raw site data are maintained with the original project.

## Archived navigation comparison

[![Indoor and outdoor comparison across the project stack, NavFn A* and NavFn Dijkstra](assets/archived-comparison.webp)](docs/navigation-comparison.md)

Two scenes, three navigation profiles, and separate measures for planned distance, inspection stops and return. [Read the results and assumptions](docs/navigation-comparison.md).

## References and discussion

Hardware and software references: [Unitree Go2](https://www.unitree.com/go2/) · [Unitree ROS 2](https://github.com/unitreerobotics/unitree_ros2) · [Unitree SDK 2](https://github.com/unitreerobotics/unitree_sdk2) · [Hesai ROS driver](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) · [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM).

Use [Discussions](https://github.com/walmfml88886666/go2-rule-based-inspection/discussions) for questions about the system and [Issues](https://github.com/walmfml88886666/go2-rule-based-inspection/issues) for reproducible problems with the public examples. Contact the maintainer before redistributing project media or code.
