"""Physically execute archived inspection waypoints in a SLAM-derived scene."""

import argparse
import hashlib
import json
import math
import signal
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from .go2_controller import Go2Controller, HOME, SIM_DT, CONTROL_DT

SCENES = {
    "indoor": (81, 109, 110, 113, 120, 2),
    "outdoor": (82, 111, 112, 115, 121, 5),
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def input_file(folder, index):
    matches = list(folder.glob(f"{index:03}_*"))
    if len(matches) != 1:
        raise ValueError(f"Expected one input with index {index:03}")
    return matches[0]


def load_inputs(folder, scene):
    ids = SCENES[scene]
    receipt = json.loads((folder / "verified-inputs.json").read_text())
    chosen = [entry for entry in receipt if int(entry["extracted"][:3]) in ids]
    if len(chosen) != len(ids) or {
        int(entry["extracted"][:3]) for entry in chosen
    } != set(ids):
        raise ValueError("The source receipt must cover every scene input exactly once")
    for entry in chosen:
        if sha(folder / entry["extracted"]) != entry["sha256"]:
            raise RuntimeError("The archived input checksum changed")
    task = json.loads(input_file(folder, ids[0]).read_text())
    terrain = np.load(input_file(folder, ids[2]))
    return task, terrain, chosen


def ground(terrain, xy):
    ix, iy = (np.asarray(xy) - terrain["origin"]) / float(terrain["resolution"])
    x, y = int(np.floor(ix)), int(np.floor(iy))
    a = terrain["ground"]
    if not (0 <= x < a.shape[1] - 1 and 0 <= y < a.shape[0] - 1):
        raise ValueError("Pose outside the archived ground surface")
    u, v = ix - x, iy - y
    return float(
        (1 - v) * ((1 - u) * a[y, x] + u * a[y, x + 1])
        + v * ((1 - u) * a[y + 1, x] + u * a[y + 1, x + 1])
    )


def build_model(inputs, scene, robot_xml):
    ids = SCENES[scene]
    original = ET.parse(input_file(inputs, ids[1])).getroot()
    root = ET.Element("mujoco", model=f"SLAM {scene}")
    ET.SubElement(root, "include", file=str(robot_xml.resolve()))
    ET.SubElement(root, "compiler", meshdir=str(robot_xml.resolve().parent / "assets"))
    ET.SubElement(root, "option", timestep=str(SIM_DT), integrator="implicitfast")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1920", offheight="1080")
    ET.SubElement(visual, "headlight", ambient=".45 .45 .45", diffuse=".55 .55 .55")
    asset = ET.SubElement(root, "asset")
    for hfield in original.findall("asset/hfield"):
        asset.append(hfield)
    world = ET.SubElement(root, "worldbody")
    for geom in original.findall("worldbody/geom"):
        if geom.get("type") == "hfield":
            geom.set("rgba", ".25 .31 .34 1")
        else:
            z = float(geom.get("pos").split()[2])
            amount = np.clip((z + 0.7) / 3, 0, 1)
            geom.set("rgba", f"{.16+.18*amount} {.42+.23*amount} {.53+.23*amount} 1")
        world.append(geom)
    xml = ET.tostring(root, encoding="unicode")
    cache = inputs.parent / "model_cache"
    cache.mkdir(exist_ok=True)
    key = hashlib.sha256(
        (
            xml + sha(robot_xml) + sha(input_file(inputs, ids[2])) + mj.__version__
        ).encode()
    ).hexdigest()
    cached = cache / (key + ".mjb")
    if cached.exists():
        return mj.MjModel.from_binary_path(str(cached))
    spec = mj.MjSpec.from_string(xml)
    body = spec.body("base") or spec.body("base_link")
    body.add_camera(
        name="inspection", pos=[0.257, 0, 0.047], xyaxes=[0, -1, 0, 0, 0, 1], fovy=56
    )
    model = spec.compile()
    terrain = np.load(input_file(inputs, ids[2]))
    assert terrain["normalized"].size == model.hfield_data.size
    model.hfield_data[:] = terrain["normalized"].ravel()
    # Check every retained terrain/obstacle primitive against the archived file.
    for geom in original.findall("worldbody/geom"):
        current = model.geom(geom.get("name"))
        np.testing.assert_allclose(
            current.pos, np.fromstring(geom.get("pos"), sep=" "), atol=1e-10
        )
        if geom.get("type") == "box":
            np.testing.assert_allclose(
                current.size, np.fromstring(geom.get("size"), sep=" "), atol=1e-10
            )
    mj.mj_saveModel(model, str(cached), None)
    return model


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def dense_waypoints(stop):
    points = np.array(
        [[w["x"], w["y"]] for w in stop["waypoints"]]
        + [[stop["observe"]["x"], stop["observe"]["y"]]]
    )
    result = [points[0]]
    for a, b in zip(points, points[1:]):
        for t in np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / 0.12) + 1))[1:]:
            result.append(a + t * (b - a))
    return np.array(result)


def run(args):
    stopping = [False]
    signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__(0, True))
    last_stop_t = 0.0
    task, terrain, source = load_inputs(args.inputs, args.scene)
    m = build_model(args.inputs, args.scene, args.robot_xml)
    d = mj.MjData(m)
    c = Go2Controller(m, args.policy)
    start = task["start_pose"]
    xy = np.array([start["x"], start["y"]])
    yaw = math.radians(start["yaw_deg"])
    d.qpos[:3] = [*xy, ground(terrain, xy) + 0.32]
    d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    d.qpos[c.qpos] = HOME
    mj.mj_forward(m, d)
    step_count = 0
    stop = 0
    route = dense_waypoints(task["stops"][stop])
    index = 0
    stable = 0
    parking = False
    phase = "stand"
    dwell_until = 0
    hold = True
    cmd = np.zeros(3)
    events = []
    states = []
    captures = []
    max_tilt = 0.0
    obstacle_samples = 0
    nonfoot_samples = 0
    previous = d.qpos[:2].copy()
    length = 0.0
    foot_ids = {m.geom(name).id for name in ("FL", "FR", "RL", "RR")}
    foot_air = np.zeros(4, dtype=int)
    foot_ground = np.zeros(4, dtype=int)
    robot_bodies = set(range(1, m.nbody))
    tick = round(CONTROL_DT / SIM_DT)
    for k in range(round(args.duration / SIM_DT)):
        t = k * SIM_DT
        rot = d.xmat[c.base].reshape(3, 3)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        tilt = math.acos(float(np.clip(rot[2, 2], -1, 1)))
        max_tilt = max(max_tilt, tilt)
        if k % tick == 0:
            xy = d.qpos[:2].copy()
            cmd[:] = 0
            hold = t < 1.5 or t < dwell_until
            if stop == len(task["stops"]) or stop == args.stop_limit:
                phase = "segment_complete"
                hold = True
                if t > dwell_until + 0.8:
                    break
            elif t < 1.5:
                phase = "stand"
            elif t < dwell_until:
                phase = "capture"
            else:
                item = task["stops"][stop]
                pose = item["observe"]
                goal = np.array([pose["x"], pose["y"]])
                error = np.linalg.norm(xy - goal)
                desired = math.radians(pose["yaw_deg"])
                if error > (0.22 if parking else 0.17):
                    parking = False
                    stable = 0
                    phase = "follow"
                    # Stay on the archived polyline; advance without skipping distant corners.
                    while (
                        index < len(route) - 1
                        and np.linalg.norm(xy - route[index]) < 0.32
                    ):
                        index += 1
                    delta = route[index] - xy
                    path_yaw = (
                        math.atan2(delta[1], delta[0]) if error > 0.55 else desired
                    )
                    local = rot[:2, :2].T @ (delta if error > 0.55 else goal - xy)
                    cmd[:2] = np.clip(local * 1.3, [-0.20, -0.22], [0.40, 0.22])
                    cmd[2] = np.clip(1.5 * wrap(path_yaw - yaw), -0.40, 0.40)
                else:
                    phase = "align"
                    angle = abs(wrap(desired - yaw))
                    if angle < math.radians(5):
                        parking = True
                    if angle > math.radians(5):
                        parking = False
                    hold = parking
                    if not hold:
                        turn = wrap(desired - yaw)
                        cmd[2] = math.copysign(
                            max(0.40, min(0.6, 1.4 * abs(turn))), turn
                        )
                    good = (
                        hold
                        and angle <= math.radians(5)
                        and np.linalg.norm(d.qvel[:2]) < 0.01
                        and abs(d.qvel[5]) < 0.06
                    )
                    stable = stable + 1 if good else 0
                    if stable >= round(0.6 / CONTROL_DT):
                        events.append(
                            dict(
                                event=(
                                    "return_arrived"
                                    if item["id"] == "RETURN_TO_START"
                                    else "capture"
                                ),
                                id=item["id"],
                                t_s=t,
                                position_error_m=float(error),
                                heading_error_deg=math.degrees(angle),
                                speed_m_s=float(np.linalg.norm(d.qvel[:2])),
                            )
                        )
                        captures.append(d.qpos.copy())
                        stop += 1
                        last_stop_t = t
                        phase = "capture"
                        parking = False
                        stable = 0
                        dwell_until = t + float(pose["photo_sec"])
                        if stop < len(task["stops"]):
                            route = dense_waypoints(task["stops"][stop])
                            index = 0
            c.command(m, d, cmd, hold)
        c.apply(m, d, hold)
        mj.mj_step(m, d)
        step_count += 1
        length += float(np.linalg.norm(d.qpos[:2] - previous))
        previous = d.qpos[:2].copy()
        touching = set()
        for ct in d.contact:
            b1, b2 = m.geom_bodyid[[ct.geom1, ct.geom2]]
            if (b1 in robot_bodies) == (b2 in robot_bodies):
                continue
            rg = ct.geom1 if b1 in robot_bodies else ct.geom2
            wg = ct.geom2 if b1 in robot_bodies else ct.geom1
            if m.geom(wg).name == "terrain":
                if rg not in foot_ids:
                    nonfoot_samples += 1
                else:
                    touching.add(rg)
            else:
                obstacle_samples += 1
        if phase == "follow":
            for j, name in enumerate(("FL", "FR", "RL", "RR")):
                if m.geom(name).id in touching:
                    foot_ground[j] += 1
                else:
                    foot_air[j] += 1
        if k % 20 == 0:
            states.append(((k + 1) * SIM_DT, d.qpos.copy(), stop, phase))
        if (
            tilt > 0.55
            or nonfoot_samples
            or obstacle_samples
            or not np.isfinite(d.qpos).all()
        ):
            events.append(
                dict(
                    event="safety_stop",
                    t_s=t,
                    body_tilt_deg=math.degrees(tilt),
                    obstacle_contact_samples=obstacle_samples,
                    non_foot_floor_contact_samples=nonfoot_samples,
                )
            )
            break
        if stopping[0] or t - last_stop_t > (240 if args.scene == "indoor" else 600):
            events.append(
                dict(event="interrupted" if stopping[0] else "stop_timeout", t_s=t)
            )
            break
        if k % 10000 == 0:
            print(
                args.scene,
                round(t),
                stop,
                len(task["stops"]),
                phase,
                "xy",
                np.round(d.qpos[:2], 2).tolist(),
                flush=True,
            )
    args.output.mkdir(parents=True, exist_ok=True)
    summary = dict(
        scene=args.scene,
        physics_dt_s=SIM_DT,
        policy_dt_s=CONTROL_DT,
        mujoco_version=mj.__version__,
        gravity_m_s2=m.opt.gravity.tolist(),
        static_geometry_count=len(
            ET.parse(input_file(args.inputs, SCENES[args.scene][1]))
            .getroot()
            .findall("worldbody/geom")
        ),
        robot_xml_sha256=sha(args.robot_xml),
        optical_view_scope="Geometric pinhole proxy; no measured camera-to-body transform or RGB texture",
        required_stops=len(task["stops"]),
        completed_stops=stop,
        required_inspection_targets=sum(
            x["id"] != "RETURN_TO_START" for x in task["stops"]
        ),
        completed_inspection_targets=sum(e["event"] == "capture" for e in events),
        return_completed=any(e["event"] == "return_arrived" for e in events),
        stop_limit=args.stop_limit,
        simulation_s=step_count * SIM_DT,
        physics_steps=step_count,
        max_body_tilt_deg=math.degrees(max_tilt),
        obstacle_contact_samples=obstacle_samples,
        non_foot_floor_contact_samples=nonfoot_samples,
        integrated_distance_m=length,
        foot_air_samples=foot_air.tolist(),
        foot_ground_samples=foot_ground.tolist(),
        events=events,
        sources=[
            dict(member=e["member"], sha256=e["sha256"], bytes=e["bytes"])
            for e in source
        ],
        policy_sha256=sha(args.policy),
        controller_sha256=sha(Path(__file__).with_name("go2_controller.py")),
        final_position=d.qpos[:3].tolist(),
        phase=phase,
    )
    (args.output / "mission.json").write_text(json.dumps(summary, indent=2) + "\n")
    np.savez_compressed(
        args.output / "states.npz",
        times=[s[0] for s in states],
        qpos=[s[1] for s in states],
        completed=[s[2] for s in states],
        phase=[s[3] for s in states],
        captures=captures,
    )
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--inputs", type=Path, required=True)
    p.add_argument("--scene", choices=SCENES, required=True)
    p.add_argument("--robot-xml", type=Path, required=True)
    p.add_argument("--policy", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--duration", type=float, default=2400)
    p.add_argument("--stop-limit", type=int, default=100)
    args = p.parse_args()
    if args.duration <= 0 or args.stop_limit < 1:
        p.error("duration and stop-limit must be positive")
    run(args)
