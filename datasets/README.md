# Indoor and outdoor LiDAR maps

| Scene | Binary XYZ PCD | Points | Route and observation poses | Stops |
| --- | --- | ---: | --- | ---: |
| Indoor | [indoor.pcd](indoor.pcd?raw=1) | 246,739 | [indoor-route.json](indoor-route.json) | 22 + return |
| Outdoor | [outdoor.pcd](outdoor.pcd?raw=1) | 1,385,388 | [outdoor-route.json](outdoor-route.json) | 8 + return |

These are geometry exports from the same frozen localization maps and original plans used in the [physical simulation](../docs/physics-patrol.md). Each scene uses its own mission start as the local origin. All finite XYZ points are retained. The export translates the coordinates without changing the axes. Units are metres and yaw is in degrees about +Z. Route coordinates are rounded to 10 µm.

PCDs contain XYZ float32 fields. The JSON includes the planned XY path and the ordered observation poses, including the return. Its pose identifiers are ordinal. The dataset scope is mapped geometry and planned routes. It excludes raw sensor logs, imagery, timestamps, site metadata, and robot configuration.

[File and source SHA-256 checksums](manifest.json) · [Rights and attribution](../LICENSE)
