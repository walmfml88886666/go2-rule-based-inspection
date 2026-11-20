"""Read and display the binary float PCD files used by the project."""

from pathlib import Path

import numpy as np

HEIGHT_COLORS = np.array(
    [[53, 105, 191], [61, 190, 181], [206, 221, 110], [252, 165, 92], [241, 106, 99]]
)
MAP_BACKGROUND = (12, 26, 40)


def read_xyz(path):
    with Path(path).open("rb") as stream:
        header = {}
        for line in stream:
            fields = line.decode("ascii").split()
            if not fields or fields[0] == "#":
                continue
            header[fields[0]] = fields[1:]
            if fields[0] == "DATA":
                break
        names = header["FIELDS"]
        if (
            header["DATA"] != ["binary"]
            or header["SIZE"] != ["4"] * len(names)
            or header["TYPE"] != ["F"] * len(names)
            or header.get("COUNT", ["1"] * len(names)) != ["1"] * len(names)
        ):
            raise ValueError(f"Unsupported PCD encoding: {path}")
        data = np.fromfile(stream, dtype="<f4").reshape(-1, len(names))
        if len(data) != int(header["POINTS"][0]):
            raise ValueError(f"PCD point count mismatch: {path}")
    xyz = data[:, [names.index(axis) for axis in "xyz"]]
    return xyz[np.isfinite(xyz).all(axis=1)]


def write_xyz(path, xyz):
    n = len(xyz)
    header = (
        f"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    with Path(path).open("wb") as output:
        output.write(header.encode("ascii"))
        xyz.astype("<f4").tofile(output)


def height_rgb(values, limits):
    positions = np.linspace(*limits, len(HEIGHT_COLORS))
    return np.stack(
        [np.interp(values, positions, channel) for channel in HEIGHT_COLORS.T], axis=-1
    ).astype(np.uint8)


def map_view(xyz, bounds, size, limits, point_radius=0):
    """Rasterize the highest measured return at each pixel in a fixed XY view."""
    width, height = size
    lower, upper = np.asarray(bounds)
    extent = np.maximum(upper - lower, 1)
    scale = min((width - 30) / extent[0], (height - 30) / extent[1])
    center = (lower + upper) / 2

    def project(xy):
        delta = np.asarray(xy) - center
        return delta * [scale, -scale] + [width / 2, height / 2]

    px = np.rint(project(xyz[:, :2])).astype(int)
    valid = (px[:, 0] >= 0) & (px[:, 0] < width) & (px[:, 1] >= 0) & (px[:, 1] < height)
    heights = np.full(width * height, -np.inf, dtype=np.float32)
    x, y = px[valid].T
    np.maximum.at(heights, y * width + x, xyz[valid, 2])
    if point_radius:
        # Grow only the rendered footprint of each return. The cloud and
        # collision geometry still use the original measurements.
        grid = heights.reshape(height, width)
        padded = np.pad(grid, point_radius, constant_values=-np.inf)
        diameter = 2 * point_radius + 1
        heights = np.maximum.reduce(
            [
                padded[y : y + height, x : x + width]
                for y in range(diameter)
                for x in range(diameter)
            ]
        ).ravel()
    occupied = np.isfinite(heights)
    rgb = np.tile(np.array(MAP_BACKGROUND, dtype=np.uint8), (width * height, 1))
    rgb[occupied] = height_rgb(heights[occupied], limits)
    return rgb.reshape(height, width, 3), project, scale
