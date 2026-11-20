"""Render a close follow-camera excerpt from the archived SLAM-map runs."""

import argparse
import json
import subprocess
from pathlib import Path
import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw
from .render_slam import BG, WHITE, MINT, AMBER, font, map_panel
from .slam_patrol import SCENES, build_model, input_file, load_inputs


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--inputs", type=Path, required=True)
    p.add_argument("--scene", choices=SCENES, required=True)
    p.add_argument("--robot-xml", type=Path, required=True)
    p.add_argument("--run", type=Path, required=True)
    p.add_argument("--ffmpeg", type=Path, required=True)
    p.add_argument("--preview", action="store_true")
    args = p.parse_args()
    task, _, _ = load_inputs(args.inputs, args.scene)
    summary = json.loads((args.run / "mission.json").read_text())
    states = np.load(args.run / "states.npz")
    times = states["times"]
    # These contiguous walking spans were chosen from the archived runs.
    start = {"indoor": 1.5, "outdoor": 110.0}[args.scene]
    spans = [(start, start + 11.0)]
    keep = np.flatnonzero((times >= spans[0][0]) & (times <= spans[0][1]))
    if args.preview:
        keep = keep[np.linspace(0, len(keep) - 1, min(6, len(keep))).astype(int)]
    model = build_model(args.inputs, args.scene, args.robot_xml)
    # MuJoCo expresses the near plane as a fraction of the scene extent.
    # The large outdoor map otherwise clips a close camera through the terrain.
    model.vis.map.znear = 0.04 / model.stat.extent
    data = mj.MjData(model)
    main = mj.Renderer(model, 684, 1040, max_geom=100000)
    options = mj.MjvOption()
    options.geomgroup[3] = False
    options.sitegroup[:] = False
    # Use a semitransparent display for the measured voxel surface. Collisions
    # retain the original geometry and parameters in the separate simulation.
    for g in range(model.ngeom):
        if model.geom(g).name.startswith("obstacle_"):
            model.geom_rgba[g, 3] = 0.5
    appearance = np.load(input_file(args.inputs, SCENES[args.scene][3]))["xyz"]
    path = np.array(task["planned_path"])
    camera = mj.MjvCamera()
    camera.type = mj.mjtCamera.mjCAMERA_FREE
    output = args.run / f"slam-{args.scene}.mp4"
    temporary = args.run / f"slam-{args.scene}.partial.mp4"
    command = [
        str(args.ffmpeg.resolve()),
        "-v",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        "1600x900",
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
        str(temporary),
    ]
    proc = None if args.preview else subprocess.Popen(command, stdin=subprocess.PIPE)
    poster = keep[len(keep) // 2]
    for number, i in enumerate(keep):
        t = float(times[i])
        xy = states["qpos"][i, :2]
        data.qpos[:] = states["qpos"][i]
        mj.mj_forward(model, data)
        # Keep a stable world orientation so the scan moves continuously past
        # the robot through turns. A close view makes individual steps visible.
        camera.lookat[:] = data.qpos[:3] + [0, 0, -0.12]
        camera.distance = 1.60
        camera.azimuth = 210
        camera.elevation = -50
        main.update_scene(data, camera=camera, scene_option=options)
        image = Image.new("RGB", (1600, 900), BG)
        draw = ImageDraw.Draw(image)

        def text(x, y, s, n, color=WHITE, bold=False):
            draw.text((x, y), s, font=font(n, bold), fill=color)

        text(38, 44, f"{args.scene.capitalize()} SLAM simulation", 55, WHITE, True)
        text(1100, 58, f"{t:06.1f} s", 37, WHITE, True)
        image.paste(Image.fromarray(main.render()), (38, 157))
        phase = str(states["phase"][i])
        completed = int(states["completed"][i])
        target_index = completed - 1 if phase == "capture" and completed else completed
        current = task["stops"][min(target_index, len(task["stops"]) - 1)]["id"]
        draw.rounded_rectangle((50, 168, 620, 217), 8, fill=BG)
        text(
            65,
            181,
            f"{phase.upper()}   /   {current}",
            30,
            AMBER if phase == "capture" else WHITE,
            True,
        )
        draw.rounded_rectangle((1099, 157, 1562, 536), 12, fill="#142b38")
        text(1115, 171, "SLAM map and recorded route", 20, WHITE, True)
        image.paste(
            map_panel(appearance[::3], path, states["qpos"][: i + 1 : 5, :2], xy),
            (1115, 208),
        )
        text(1115, 512, "Plan", 21, AMBER, True)
        text(1210, 512, "Driven", 21, MINT, True)
        draw.rounded_rectangle((1099, 560, 1562, 841), 12, fill="#142b38")
        returned = summary["return_completed"]
        reached = summary["completed_inspection_targets"]
        total = summary["required_inspection_targets"]
        text(1125, 577, "MISSION RESULT", 26, WHITE, True)
        text(
            1125, 625, f"{reached}/{total} stops", 58, MINT if returned else AMBER, True
        )
        text(
            1125,
            697,
            "Returned to start" if returned else "Safety stop",
            30,
            WHITE,
            True,
        )
        if i == poster:
            image.save(args.run / f"slam-{args.scene}.jpg", quality=93)
        if args.preview:
            image.save(args.run / f"preview-{number}.jpg", quality=90)
        else:
            proc.stdin.write(image.tobytes())
        if number % 250 == 0:
            print(args.scene, "render", number, len(keep), flush=True)
    if proc is not None:
        proc.stdin.close()
        if proc.wait():
            raise RuntimeError("Video encoding failed")
        temporary.replace(output)
    (
        args.run
        / ("preview-selection.json" if args.preview else "render-selection.json")
    ).write_text(
        json.dumps(
            dict(spans=spans, frames=len(keep), mode="excerpts", playback_speed=1),
            indent=2,
        )
    )
    print(output, flush=True)


if __name__ == "__main__":
    main()
