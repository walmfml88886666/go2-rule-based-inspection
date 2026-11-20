# XT16 mapping adaptation and planning layers

The field system uses the ROS 2 Humble port of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM). The project's XT16 configuration adapts the sensor-to-body transform and roll/pitch fusion to the mounted Go2 system.

## Matched indoor mapping comparison

Both runs processed the first 1,040 s of the same indoor LiDAR and IMU recording, using the same executable. The input rotation and the IMU roll/pitch fusion weight changed together. The white line in the figure is the archived SLAM trajectory projected along the same horizontal axis in both panels. Surface returns are colored by map Z using one palette and the same limits. The plot exaggerates the vertical axis by 4×.

| Configuration | IMU vector rotation | Roll/pitch weight | Keyframes | Estimated trajectory Z span |
| --- | --- | ---: | ---: | ---: |
| Baseline | Identity | 0.01 | 1,162 | 5.966 m |
| Calibrated | −90° about Z | 0.15 | 1,145 | 0.891 m |

The Z span is the difference between the maximum and minimum estimated trajectory elevations. It summarizes this paired run. Absolute height error would require an independent reference. The two changed settings were not separately ablated. [Measured values and source digests](data/mapping-calibration-ab.json)

## Planner implementation

The delivered indoor and outdoor `inspection_planner.py` compute a local ground surface on a 0.15 m XY grid. For a populated cell within 3.0 m of the nearest mapping pose, the algorithm subtracts the 0.485 m LiDAR mount height from that pose and allows elevation change up to a 25° slope plus 0.16 m step and 0.05 m noise margins. It then takes the 15th percentile of returns below that ceiling. Cells with no surviving returns use the trajectory ground estimate within the supported radius. Cells beyond the radius use the uncapped percentile, and empty cells are unknown at the estimator stage.

The navigation mask counts returns within (0.16, 0.6296) m above the estimated ground. The upper limit is the sum of the 0.485 m mount height, 0.0296 m offset to the LiDAR cap, and 0.115 m clearance margin. The visibility mask counts returns within (0.90, 2.20) m. Each raw mask activates at two returns. The navigation layer can be inflated for footprint clearance; the view layer supports ray sampling from an observation pose towards a target and stops short of the target itself. These are planning rules for the project's geometry model.

The route uses Dijkstra graph distances for waypoint legs and applies visit-order and observation checks separately. [System workflow](system.md)
