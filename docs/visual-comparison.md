# Rendering review

## System architecture

![Architecture](../assets/inspection-architecture.png)

## Planning layers

![Layers](../assets/navigation-sight-layers.png)

## Measured 3D environment candidates

The original point positions are height-colored and viewed from oblique cameras. Left and right columns compare 35° and 65° camera elevations. These are visualization candidates from the original localization maps.

![3D candidates](../assets/scene-render-comparison.jpg)

## Second round

The architecture uses a common light background and a roof cutaway. The two layers have more separation, physical obstacle examples, and an explicit grid pitch. The measured scene views below use 35° oblique cameras, display-only sparse-cell filtering and per-scene height colors.

![Indoor cutaway](../assets/indoor-scene-3d.png)
![Outdoor scene](../assets/outdoor-scene-3d.png)
