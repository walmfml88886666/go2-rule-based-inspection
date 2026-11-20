# Go2 Patrol

Rule-based LiDAR SLAM inspection with Unitree Go2 EDU. The system plans routes with Dijkstra and links each field run to its trajectory and inspection photos.

## Field demos

| Indoor inspection | Outdoor gravel crossing |
| :---: | :---: |
| [![Go2 walking to an indoor inspection target](assets/field-exterior-indoor.webp)](assets/field-exterior-indoor.mp4?raw=1) | [![Go2 walking from pavement onto gravel](assets/field-exterior-outdoor.webp)](assets/field-exterior-outdoor.mp4?raw=1) |

### Indoor mission replay · 22/22 captures

[![Full recorded indoor trajectory and capture timeline through 22 of 22](assets/indoor-mission-replay.webp)](https://github.com/walmfml88886666/go2-rule-based-inspection/releases/download/v0.3.0/indoor-mission-replay.mp4)

## SLAM-map physical simulation

The Go2 follows the original inspection routes on LiDAR-derived terrain in MuJoCo. The previews show continuous walking at 1× speed.

**Outdoor · 8/8 inspection stops and return**

[![Close view of Go2 walking on the outdoor SLAM map](assets/slam-outdoor-walk.webp)](assets/slam-outdoor-walk.mp4?raw=1)

**Indoor · 4/22 inspection stops before a contact-triggered safety stop**

[![Close view of Go2 walking on the indoor SLAM map](assets/slam-indoor-walk.webp)](assets/slam-indoor-walk.mp4?raw=1)

## System architecture

[![Go2 EDU, Hesai XT16 and the mapping-to-inspection workflow](assets/architecture.svg)](docs/system.md)

## Navigation comparison

[![Recorded indoor and outdoor comparison of the project stack, NavFn A* and NavFn Dijkstra](assets/archived-comparison.webp)](docs/navigation-comparison.md)

## Run the route audit

```bash
python -m pip install -r requirements.txt
python -m examples.offline_audit
```
