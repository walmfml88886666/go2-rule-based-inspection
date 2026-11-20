"""Re-render archived MuJoCo poses inside the measured point-cloud scene."""

import argparse
import json
import os
from pathlib import Path
import subprocess

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw
from .measured_renderer import MeasuredRenderer, PALETTE
from .render_slam import BG, WHITE, MINT, AMBER, font
from .slam_patrol import SCENES, build_model, ground, input_file, load_inputs
from tools.point_cloud import read_xyz, height_rgb


class SceneContext:
    """Fixed camera, common XYZ frame and depth ordering for the context inset."""

    def __init__(self, cloud, route, terrain, scene):
        self.surface = {k: terrain[k] for k in ("origin", "resolution", "ground")}
        limits = np.percentile(cloud[:, 2], [5, 95])
        low, high = route.min(0) - 4, route.max(0) + 4
        center = np.r_[(low + high) / 2, np.percentile(cloud[:, 2], 15)]
        angle = 35 if scene == "indoor" else 55
        self.eye = center + max(high - low) * np.array(
            [0.55, -0.75, np.tan(np.deg2rad(angle)) * 0.6]
        )
        forward = center - self.eye
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0, 0, 1])
        right /= np.linalg.norm(right)
        self.rotation = np.array([right, np.cross(right, forward), -forward])
        valid = ((cloud[:, :2] >= low) & (cloud[:, :2] <= high)).all(1)
        if scene == "indoor":
            valid &= cloud[:, 2] < limits[0] + 1.9
        selected = cloud[valid]
        camera_xyz = (selected - self.eye) @ self.rotation.T
        uv = camera_xyz[:, :2] / -camera_xyz[:, 2:]
        uvlow, uvhigh = np.percentile(uv, [1, 99], axis=0)
        self.focal = min(
            465 / np.ptp([uvlow[0], uvhigh[0]]), 360 / np.ptp([uvlow[1], uvhigh[1]])
        )
        self.offset = np.array([250, 205]) - (uvlow + uvhigh) / 2 * [
            self.focal,
            -self.focal,
        ]
        pix = np.rint(uv * [self.focal, -self.focal] + self.offset).astype(int)
        keep = (
            (pix[:, 0] >= 0)
            & (pix[:, 0] < 500)
            & (pix[:, 1] >= 0)
            & (pix[:, 1] < 410)
            & (camera_xyz[:, 2] < 0)
        )
        order = np.argsort(camera_xyz[keep, 2])
        u, v = pix[keep][order].T
        rgb = np.full((410, 500, 3), [12, 26, 40], dtype=np.uint8)
        rgb[v, u] = height_rgb(selected[keep][order, 2], limits)
        self.base = Image.fromarray(rgb)

    def project(self, xy):
        xy = np.atleast_2d(xy)
        xyz = np.c_[xy, [ground(self.surface, p) + 0.18 for p in xy]]
        camera_xyz = (xyz - self.eye) @ self.rotation.T
        return (
            camera_xyz[:, :2] / -camera_xyz[:, 2:] * [self.focal, -self.focal]
            + self.offset
        )

    def render(self, route, trace, goal, heading):
        image = self.base.copy()
        d = ImageDraw.Draw(image)
        d.line([tuple(p) for p in self.project(route)], fill="#657a8b", width=2)
        if len(trace) > 1:
            d.line([tuple(p) for p in self.project(trace)], fill=MINT, width=4)
        x, y = self.project(goal)[0]
        d.ellipse((x - 7, y - 7, x + 7, y + 7), fill=AMBER)
        x, y = self.project(trace[-1])[0]
        direction = self.project(trace[-1] + 0.3 * heading)[0] - [x, y]
        theta = np.arctan2(direction[1], direction[0])
        angles = theta + np.array([0, 2.45, -2.45])
        arrow = np.c_[np.cos(angles), np.sin(angles)] * np.array([[16], [10], [10]]) + [
            x,
            y,
        ]
        d.polygon([tuple(p) for p in arrow], fill=WHITE, outline=BG)
        d.text(
            (x - 20, y + 15),
            "Go2",
            font=font(32, True),
            fill=WHITE,
            stroke_width=2,
            stroke_fill=BG,
        )
        return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("inputs", "robot-xml", "run", "output", "ffmpeg"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--scene", choices=SCENES, required=True)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    task, terrain, _ = load_inputs(args.inputs, args.scene)
    summary = json.loads((args.run / "mission.json").read_text())
    states = np.load(args.run / "states.npz")
    times, qpos = states["times"], states["qpos"]
    start = 2.0 if args.scene == "indoor" else 110.0
    keep = np.flatnonzero((times >= start) & (times <= start + 11))
    cloud = read_xyz(input_file(args.inputs, SCENES[args.scene][-1]))
    model = build_model(args.inputs, args.scene, args.robot_xml)
    data = mj.MjData(model)
    renderer = MeasuredRenderer(model, cloud, qpos[keep], args.scene)
    route = np.asarray(task["planned_path"])[:, :2]
    context = SceneContext(cloud, route, terrain, args.scene)
    if args.preview:
        keep = keep[np.linspace(0, len(keep) - 1, 6).astype(int)]
    output = args.output / f"slam-{args.scene}-route.mp4"
    command = [
        str(args.ffmpeg),
        "-v",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        "1800x1120",
        "-framerate",
        "25",
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-threads",
        "3",
        "-preset",
        "fast",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output),
    ]
    encoder = None if args.preview else subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        for number, i in enumerate(keep):
            data.qpos[:] = qpos[i]
            mj.mj_forward(model, data)  # Pose kinematics only; never step physics.
            recent = np.vstack([qpos[max(0, i - 175) : i + 1 : 3, :2], qpos[i, :2]])
            trace = np.c_[recent, [ground(context.surface, p) + 0.055 for p in recent]]
            image = Image.new("RGB", (1800, 1120), BG)
            image.paste(Image.fromarray(renderer.render(data, trace)), (30, 135))
            d = ImageDraw.Draw(image)
            d.text(
                (35, 22),
                f"{args.scene.capitalize()} inspection · measured scene + simulated Go2",
                font=font(39, True),
                fill=WHITE,
            )
            d.text(
                (35, 83),
                "3D / Measured XYZ surfels · archived MuJoCo poses · 1× playback",
                font=font(25),
                fill=WHITE,
            )
            d.text((1480, 84), f"{times[i]:.1f} s", font=font(27, True), fill=WHITE)
            completed = int(states["completed"][i])
            stop = task["stops"][min(completed, len(task["stops"]) - 1)]
            goal = np.array([stop["observe"][axis] for axis in "xy"])
            heading = data.xmat[
                model.cam_bodyid[model.camera("inspection").id]
            ].reshape(3, 3)[:2, 0]
            inset = context.render(
                route, np.vstack([qpos[: i + 1 : 5, :2], qpos[i, :2]]), goal, heading
            )
            d.rectangle((51, 155, 411, 495), fill=BG, outline="#50718a", width=2)
            d.text((61, 165), "3D / Mission context", font=font(25, True), fill=WHITE)
            image.paste(inset.resize((340, 279), Image.Resampling.LANCZOS), (61, 207))
            d = ImageDraw.Draw(image)
            d.rounded_rectangle((1290, 884, 1747, 982), radius=10, fill=BG)
            d.text(
                (1306, 893), "Height above ground ref (m)", font=font(26), fill=WHITE
            )
            for k in range(320):
                position = k / 319 * 4
                j = min(3, int(position))
                colors = np.array(
                    [[int(c[a : a + 2], 16) for a in (1, 3, 5)] for c in PALETTE]
                )
                rgb = tuple(
                    (
                        colors[j] * (1 - (position - j))
                        + colors[j + 1] * (position - j)
                    ).astype(int)
                )
                d.line((1310 + k, 934, 1310 + k, 945), fill=rgb)
            d.text(
                (1308, 949), "0                  2 (clipped)", font=font(24), fill=WHITE
            )
            full = f"{summary['completed_inspection_targets']}/{summary['required_inspection_targets']} stops"
            outcome = (
                "returned to start"
                if summary["completed_inspection_targets"]
                == summary["required_inspection_targets"]
                else "contact-triggered safety stop"
            )
            distance = np.linalg.norm(goal - qpos[i, :2])
            d.text(
                (35, 1031),
                f"Next: {stop['id']} · {distance:.1f} m straight-line",
                font=font(27, True),
                fill=AMBER,
            )
            d.text(
                (770, 1031),
                f"FULL RUN: {full} · {outcome}",
                font=font(27, True),
                fill=MINT,
            )
            d.text(
                (35, 1080),
                "Mint: motion · Amber: next stop in inset · Display surfels ≠ collision geometry; archived states unchanged.",
                font=font(24),
                fill="#c5d5e0",
            )
            if args.preview or i == keep[len(keep) // 2]:
                image.save(
                    args.output / f"{args.scene}-measured-{number:03}.jpg", quality=93
                )
            if i == keep[len(keep) // 2]:
                image.save(args.output / f"slam-{args.scene}.jpg", quality=93)
            if encoder:
                encoder.stdin.write(image.tobytes())
            if number % 50 == 0:
                print(args.scene, number, len(keep), flush=True)
    finally:
        renderer.close()
        if encoder:
            encoder.stdin.close()
    if encoder and encoder.wait():
        raise RuntimeError("Video encoding failed")
    (args.output / f"{args.scene}-display.json").write_text(
        json.dumps(
            dict(
                point_count=renderer.point_count,
                ground_reference_z_m=renderer.floor,
                height_span_m=renderer.height_span,
                surfel_radius_range_m=[0.045, 0.095],
                surfel_radius_rule="clip(0.7 * distance_to_4th_neighbor, 0.045, 0.095)",
                edl_weight=0.15,
                pca_neighbors=16,
                displayed_time_s=[float(times[keep[0]]), float(times[keep[-1]])],
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
