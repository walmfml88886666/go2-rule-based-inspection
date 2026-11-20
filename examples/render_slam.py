"""Render selected, timestamped states from a SLAM-map physics experiment."""

import argparse, json, math, subprocess
from pathlib import Path
import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from .slam_patrol import build_model, load_inputs, input_file, SCENES

BG = "#0b1721"
WHITE = "#eaf1f4"
MUTED = "#afc3ce"
MINT = "#55e3c4"
AMBER = "#ffbd6d"


def font(n, bold=False):
    return ImageFont.truetype(f"DejaVuSans{'-Bold' if bold else ''}.ttf", n)


def selection(times, events):
    end = float(times[-1])
    spans = [(0, min(12, end))]
    captures = [e for e in events if e["event"] == "capture"]
    if captures:
        for event in (captures[0], captures[-1]):
            spans.append((max(0, event["t_s"] - 5), min(end, event["t_s"] + 3)))
    else:
        spans.append((max(0, end - 12), end))
    if end > 60:
        spans.append((end * 0.45, min(end, end * 0.45 + 8)))
    spans.append((max(0, end - 5), end))
    keep = np.zeros(len(times), bool)
    for a, b in spans:
        keep |= (times >= a) & (times <= b)
    return np.flatnonzero(keep), spans


def map_panel(points, path, trace, xy):
    result = Image.new("RGB", (432, 304), "#112837")
    array = np.asarray(result).copy()

    # The local panel spans 16 x 11.3 metres of the actual map.
    def project(a):
        v = np.asarray(a)
        return np.column_stack(
            ((v[:, 0] - xy[0]) * 27 + 216, 152 - (v[:, 1] - xy[1]) * 27)
        )

    pix = project(points)
    valid = (pix[:, 0] >= 0) & (pix[:, 0] < 432) & (pix[:, 1] >= 0) & (pix[:, 1] < 304)
    q = pix[valid].astype(int)
    z = points[valid, 2]
    z = np.clip((z - np.percentile(points[:, 2], 5)) / 2.5, 0, 1)
    rgb = np.column_stack((35 + 45 * z, 85 + 70 * z, 105 + 75 * z)).astype(np.uint8)
    array[q[:, 1], q[:, 0]] = rgb
    result = Image.fromarray(array)
    draw = ImageDraw.Draw(result)
    for a, color, width in ((path, AMBER, 2), (trace, MINT, 3)):
        if len(a) > 1:
            draw.line([tuple(v) for v in project(a)], fill=color, width=width)
    draw.ellipse((211, 147, 221, 157), fill=WHITE)
    return result


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
    keep, spans = selection(times, summary["events"])
    if args.preview:
        keep = keep[np.linspace(0, len(keep) - 1, min(6, len(keep))).astype(int)]
    model = build_model(args.inputs, args.scene, args.robot_xml)
    data = mj.MjData(model)
    main = mj.Renderer(model, 684, 1040, max_geom=100000)
    lens = mj.Renderer(model, 244, 432, max_geom=100000)
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
    captures = [
        e for e in summary["events"] if e["event"] in ("capture", "return_arrived")
    ]
    capture_frames = {
        int(abs(times - e["t_s"]).argmin()): j for j, e in enumerate(captures)
    }
    # Export each capture directly at its saved physical pose.
    for j, event in enumerate(captures):
        if event["event"] != "capture":
            continue
        data.qpos[:] = states["captures"][j]
        mj.mj_forward(model, data)
        lens.update_scene(data, camera="inspection", scene_option=options)
        Image.fromarray(lens.render()).save(args.run / f"capture-{event['id']}.png")
    first_capture = next(
        (e["t_s"] for e in summary["events"] if e["event"] == "capture"),
        float(times[keep[len(keep) // 3]]),
    )
    poster = keep[abs(times[keep] - min(times[-1], first_capture + 1)).argmin()]
    for number, i in enumerate(keep):
        t = float(times[i])
        xy = states["qpos"][i, :2]
        data.qpos[:] = states["qpos"][i]
        mj.mj_forward(model, data)
        q = data.qpos[3:7]
        yaw = math.degrees(
            math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        )
        camera.lookat[:] = data.qpos[:3] + [0.15, 0, 0.05]
        camera.distance = 3.3
        look = camera.lookat.copy()
        chosen = False
        for elevation in (-38, -55, -72):
            if chosen:
                break
            for angle in (145, 90, 180, 225, 45, 270, 0, 315):
                azimuth = yaw + angle
                az = math.radians(azimuth)
                el = math.radians(elevation)
                eye = look + 3.3 * np.array(
                    [
                        math.cos(el) * math.cos(az),
                        math.cos(el) * math.sin(az),
                        -math.sin(el),
                    ]
                )
                clear = True
                targets = [
                    data.qpos[:3]
                    + data.xmat[
                        model.cam_bodyid[model.camera("inspection").id]
                    ].reshape(3, 3)[:, 0]
                    * offset
                    for offset in (-0.20, 0, 0.20)
                ]
                targets += [
                    data.geom_xpos[model.geom(leg).id]
                    for leg in ("FL", "FR", "RL", "RR")
                ]
                for target in targets:
                    delta = target - eye
                    length = np.linalg.norm(delta)
                    hit = mj.mj_ray(
                        model,
                        data,
                        eye,
                        delta / length,
                        np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8),
                        1,
                        -1,
                        np.zeros(1, dtype=np.int32),
                    )
                    if 0 <= hit < length - 0.06:
                        clear = False
                        break
                if clear:
                    camera.azimuth = azimuth
                    camera.elevation = elevation
                    chosen = True
                    break
        if not chosen:
            camera.distance = 1.8
            camera.azimuth = yaw + 145
            camera.elevation = -78
        main.update_scene(data, camera=camera, scene_option=options)
        lens.update_scene(data, camera="inspection", scene_option=options)
        image = Image.new("RGB", (1600, 900), BG)
        draw = ImageDraw.Draw(image)

        def text(x, y, s, n, color=WHITE, bold=False):
            draw.text((x, y), s, font=font(n, bold), fill=color)

        text(38, 25, "GO2 PATROL   /   PROJECT SLAM DATA", 20, MINT, True)
        text(
            36,
            56,
            f"{args.scene.capitalize()} inspection on the recorded map",
            41,
            WHITE,
            True,
        )
        text(
            38,
            114,
            "Archived LiDAR scene + inspection route   /   Torque-driven Go2   /   Simulator pose feedback",
            18,
            MUTED,
        )
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
            22,
            AMBER if phase == "capture" else WHITE,
            True,
        )
        draw.rounded_rectangle((1099, 157, 1562, 536), 12, fill="#142b38")
        text(1115, 171, "SLAM map and recorded route", 20, WHITE, True)
        image.paste(
            map_panel(appearance[::3], path, states["qpos"][: i + 1 : 5, :2], xy),
            (1115, 208),
        )
        text(1115, 516, "Amber: planned   Teal: simulated", 15, MUTED)
        draw.rounded_rectangle((1099, 548, 1562, 841), 12, fill="#142b38")
        text(1115, 559, "Virtual camera · no RGB", 19, WHITE, True)
        image.paste(Image.fromarray(lens.render()), (1115, 592))
        text(
            38,
            854,
            f"{t:07.2f} s SIM TIME   /   Selected excerpts at 1x   /   Scan geometry · no measured camera extrinsics",
            17,
            MUTED,
        )
        if i == poster:
            image.save(args.run / f"slam-{args.scene}.jpg", quality=93)
        if args.preview:
            image.save(args.run / f"preview-{number}.jpg", quality=90)
        else:
            proc.stdin.write(image.tobytes())
        if number % 250 == 0:
            print(args.scene, "render", number, len(keep), flush=True)
    if not args.preview:
        ending = summary["events"][-1] if summary["events"] else {}
        if ending.get("event") in ("safety_stop", "stop_timeout", "interrupted"):
            image = Image.new("RGB", (1600, 900), BG)
            draw = ImageDraw.Draw(image)
            lines = [
                ("PROJECT SLAM DATA / RECORDED OUTCOME", 30, MINT),
                (f"{args.scene.capitalize()} simulation", 54, WHITE),
                (
                    f'{summary["completed_inspection_targets"]} / {summary["required_inspection_targets"]} inspection poses reached',
                    37,
                    WHITE,
                ),
                (
                    f'Stopped at {summary["simulation_s"]:.2f} simulated seconds',
                    30,
                    MUTED,
                ),
                (
                    f'Obstacle contact samples: {summary["obstacle_contact_samples"]}',
                    30,
                    AMBER,
                ),
                (
                    f'Non-foot floor contact samples: {summary["non_foot_floor_contact_samples"]}',
                    30,
                    MUTED,
                ),
                ("See the run record for all inputs and capture errors.", 25, MUTED),
            ]
            y = 135
            for text, size, color in lines:
                draw.text((90, y), text, font=font(size, True), fill=color)
                y += 85
            for _ in range(75):
                proc.stdin.write(image.tobytes())
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
