"""Render logged MuJoCo states and the chassis-mounted camera."""
import argparse
import json
from pathlib import Path
import subprocess

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .physics_patrol import CART_X, CART_HALF, FIXTURES, STOPS, build_scene


BG = "#0c1821"
PANEL = "#132633"
WHITE = "#f0f5f5"
MUTED = "#a7bdc8"
MINT = "#52e8c7"
AMBER = "#ffb35e"


def font(size, bold=False):
    suffix = "-Bold" if bold else ""
    return ImageFont.truetype(f"DejaVuSans{suffix}.ttf", size)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("robot_xml", type=Path)
    parser.add_argument("ffmpeg", type=Path)
    args = parser.parse_args()
    run = json.loads((args.run/"mission.json").read_text())
    if not run["completed_capture_sequence"]:
        raise SystemExit("The simulated mission must pass before rendering")
    states = np.load(args.run/"state.npz")
    model = build_scene(args.robot_xml)
    if model.nmocap != 1:
        raise ValueError("Expected exactly one scripted cart")
    data = mj.MjData(model)
    main_view = mj.Renderer(model, 684, 1040)
    fpv = mj.Renderer(model, 244, 432)
    camera = mj.MjvCamera()
    camera.type = mj.mjtCamera.mjCAMERA_FREE
    options = mj.MjvOption()
    options.geomgroup[3] = False
    options.sitegroup[:] = False
    times = states["times"]
    path_xy = states["qpos"][:, :2]
    cap_events = [e for e in run["events"] if e["event"] == "capture"]
    poster = abs(times - (cap_events[0]["t_s"] + .4)).argmin()
    capture_frames = {int(abs(times - e["t_s"]).argmin()): e for e in cap_events}
    args.run.mkdir(exist_ok=True, parents=True)
    mp4 = args.run/"go2-physics-patrol.mp4"
    command = [str(args.ffmpeg.resolve()), "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pixel_format",
               "rgb24", "-video_size", "1600x900", "-framerate", "25", "-i", "-",
               "-an", "-c:v", "libx264", "-threads", "3", "-preset", "fast", "-crf", "20",
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        for i, t in enumerate(times):
            if i % 2 and i not in capture_frames and i != poster:
                continue
            data.qpos[:] = states["qpos"][i]
            data.mocap_pos[:] = states["mocap"][i]
            mj.mj_forward(model, data)
            phase = states["phase"][i]
            captured = sum(e["t_s"] <= t for e in cap_events)
            cart_y = data.mocap_pos[0, 1]
            overhead = t < 2.4
            camera.lookat[:] = [3.8, .2, 0] if overhead else [data.qpos[0]+.40, data.qpos[1]+.10, .38]
            camera.distance = 13.4 if overhead else (2.85 if phase in ("align","capture","complete") else 3.5)
            camera.azimuth = (110 if overhead else 125) + 9*np.sin(t*.16)
            camera.elevation = -58 if overhead else -21
            main_view.update_scene(data, camera=camera, scene_option=options)
            fpv.update_scene(data, camera="inspection", scene_option=options)
            panel = Image.fromarray(main_view.render())
            lens = Image.fromarray(fpv.render())
            if i in capture_frames:
                number = capture_frames[i]["stop"]
                captured_data = mj.MjData(model)
                captured_data.qpos[:] = states["capture_qpos"][number-1]
                captured_data.mocap_pos[:] = states["mocap"][i]
                mj.mj_forward(model, captured_data)
                fpv.update_scene(captured_data, camera="inspection", scene_option=options)
                Image.fromarray(fpv.render()).save(args.run/f"simulated-capture-{number}.png")
            image = Image.new("RGB", (1600,900), BG)
            draw = ImageDraw.Draw(image)
            draw.text((38,28), "GO2 PATROL   /   SIMULATED WORKSHOP", font=font(19,True), fill=MINT)
            draw.text((36,56), "Aisle detour and inspection stops", font=font(44,True), fill=WHITE)
            draw.text((38,115), "Original GridPath (Dijkstra)   /   MuJoCo torque control   /   Simulated RGB camera", font=font(18), fill=MUTED)
            image.paste(panel,(38,157))
            draw.rounded_rectangle((51,170,465,217), 7, fill=BG)
            status = {"stand":"Starting stance", "follow":"Following the planned route", "align":"Aligning and settling",
                      "capture":"Simulated photo captured", "complete":"Inspection segment complete"}[phase]
            draw.text((65,181), status, font=font(22,True), fill=AMBER if phase == "capture" else WHITE)
            draw.rounded_rectangle((1099,157,1562,508), 12, fill=PANEL)
            draw.text((1114,174),"Dijkstra route and simulated pose",font=font(20,True),fill=WHITE)
            scale = 31
            def xy(point):
                return (1120+(point[0]+1.65)*scale, 342-point[1]*scale)
            def rect(x,y,a,b,color):
                draw.rectangle((*xy((x-a,y+b)), *xy((x+a,y-b))), fill=color)
            for x,y,a,b in FIXTURES:
                if a < 6 and b < 3:
                    rect(x,y,a,b,"#365464")
            rect(CART_X,cart_y,*CART_HALF,AMBER)
            if t >= 4:
                draw.line([xy(p) for p in run["plans"][0]["xy"]],fill="#8c7277",width=2)
            active = [p for p in run["plans"] if p["t_s"] <= t][-1]
            draw.line([xy(p) for p in active["xy"]],fill=MINT,width=4)
            if i:
                draw.line([xy(p) for p in path_xy[:i+1]],fill="#719af5",width=2)
            for number, p in enumerate(STOPS,1):
                x,y=xy(p)
                draw.ellipse((x-9,y-9,x+9,y+9),fill=MINT if captured >= number else "#4a646f")
                draw.text((x-4,y-8),str(number),font=font(12,True),fill=BG if captured >= number else WHITE)
            x,y=xy(path_xy[i])
            draw.ellipse((x-5,y-5,x+5,y+5),fill=WHITE)
            draw.text((1115,471), "Teal: plan   Blue: trace   Amber: cart",font=font(16),fill=MUTED)
            draw.rounded_rectangle((1099,520,1562,841),12,fill=PANEL)
            draw.text((1115,536),"Go2 chassis camera",font=font(20,True),fill=WHITE)
            draw.text((1486,538),f"{captured} / 2",font=font(18,True),fill=MINT)
            image.paste(lens,(1115,577))
            if phase == "capture":
                draw.rectangle((1115,577,1546,820), outline=MINT, width=4)
            draw.text((39,858), f"{t:05.1f} s SIM TIME     /     2× playback     /     Cart crossing triggered by a scenario event",
                      font=font(16),fill=MUTED)
            if i == poster:
                image.save(args.run/"go2-physics-patrol.jpg",quality=92)
            if i % 2 == 0:
                process.stdin.write(image.tobytes())
    finally:
        process.stdin.close()
        status = process.wait()
    if status:
        raise SystemExit(status)
    print(mp4)


if __name__ == "__main__":
    main()
