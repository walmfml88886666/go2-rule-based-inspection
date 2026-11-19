"""Run a synthetic inspection with the original project's GridPath and MuJoCo."""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
import onnxruntime as ort


CELL = 0.15
ORIGIN = np.array([-1.975, -3.975])
START = (0.05, 0.0)
STOPS = ((8.0, 0.0), (8.0, 2.25))
TARGETS = ((9.35, 0.0), (9.35, 2.25))
CART_X = 4.55
CART_HALF = (0.52, 0.57)
# Physical rectangles, expressed as center x/y and half x/y dimensions.
FIXTURES = ((5, -3.85, 6.8, .15), (5, 3.85, 6.8, .15),
            (-1.75, 0, .15, 3.9), (11.75, 0, .15, 3.9),
            (1.4, 2.55, 1.3, .62), (3.5, -2.6, 1.1, .52),
            (6.9, -2.7, .9, .6), (10.15, 2.45, .65, .95),
            (9.52, 0, .14, .47), (9.52, 2.25, .14, .47))
ROBOT_LEGS = ("FR", "FL", "RR", "RL")
HOME = np.tile([0., .8, -1.5], 4)
ACTION_SCALE = np.tile([.125, .25, .25], 4)
SIM_DT, CONTROL_DT = .005, .02


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_planner(path):
    spec = importlib.util.spec_from_file_location("original_inspection_planner", path)
    if spec is None or spec.loader is None:
        raise ValueError("Provide the field project inspection_planner.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.GridPath


class SceneGrid:
    cell = CELL
    min_xy = ORIGIN

    def __init__(self, cart_y):
        y, x = np.indices((53, 92)) * CELL + CELL / 2 + ORIGIN[::-1, None, None]
        self.scanned = np.ones(x.shape, dtype=bool)
        self.obstacle = np.zeros(x.shape, dtype=bool)
        for cx, cy, hx, hy in (*FIXTURES, (CART_X, cart_y, *CART_HALF)):
            self.obstacle |= (abs(x - cx) <= hx + .48) & (abs(y - cy) <= hy + .48)

    def to_idx(self, x, y):
        return tuple(np.floor((np.array([x, y]) - ORIGIN) / CELL).astype(int))


def plan(grid_path, begin, end, cart_y):
    grid = SceneGrid(cart_y)
    graph = grid_path(grid)
    distances, nodes, pred = graph.solve([begin, end])
    if distances[0, 1] >= 1e6:
        raise RuntimeError("The original planner could not connect the two stops")
    path = np.asarray(graph.reconstruct(pred[0], *nodes))
    for x, y in path:
        ix, iy = grid.to_idx(x, y)
        if grid.obstacle[iy, ix]:
            raise RuntimeError("The center route intersects the inflated obstacle mask")
    return path


def node(parent, kind, **args):
    return ET.SubElement(parent, kind, {key: str(val) for key, val in args.items()})


def xyz(values):
    return " ".join(f"{v:.5f}" for v in values)


def build_scene(robot_xml):
    root = ET.Element("mujoco", model="Go2 inspection workshop")
    node(root, "include", file=robot_xml.resolve())
    node(root, "compiler", meshdir=robot_xml.resolve().parent / "assets")
    node(root, "option", timestep=SIM_DT, integrator="implicitfast")
    visual = node(root, "visual")
    node(visual, "global", offwidth=1920, offheight=1080)
    node(visual, "quality", shadowsize=2048)
    node(visual, "headlight", ambient=".32 .33 .35", diffuse=".50 .49 .46", specular=".18 .18 .18")
    assets = node(root, "asset")
    node(assets, "texture", name="shop_tiles", type="2d", builtin="checker",
         rgb1=".24 .27 .29", rgb2=".20 .23 .25", width=512, height=512)
    node(assets, "material", name="shop_floor", texture="shop_tiles", texrepeat="18 10", reflectance=.08, roughness=.72)
    world = node(root, "worldbody")
    node(world, "geom", name="floor", type="plane", size="13 9 .1", material="shop_floor", friction=".8 .05 .01")
    for x in (0, 4.5, 9):
        node(world, "light", pos=f"{x} -1.8 5", dir=".15 .27 -1", diffuse=".68 .66 .61",
             directional="false", castshadow="true")

    def box(parent, name, pos, half, color, decor=False):
        return node(parent, "geom", name=name, type="box", pos=xyz(pos), size=xyz(half),
                    rgba=color, contype=0 if decor else 1, conaffinity=0 if decor else 1)

    for i, (x, y, a, b) in enumerate(FIXTURES):
        height = .54 if i < 4 else (.78 if i < 8 else .42)
        box(world, f"fixture_{i}", (x, y, height), (a, b, height),
            ".31 .36 .40 1" if i < 4 else ".17 .33 .40 1")
        box(world, f"fixture_top_{i}", (x, y, height * 2 - .025), (a+.005, b+.005, .035), ".09 .16 .21 1", True)
        if 4 <= i < 8:
            for sign in (-1, 1):
                box(world, f"fixture_plate_{i}_{sign}", (x+.42*sign, y, height), (.26, b+.018, .21), ".42 .49 .51 1", True)
                for j in range(4):
                    box(world, f"fixture_vent_{i}_{sign}_{j}", (x+.42*sign, y, height-.11+.07*j),
                        (.20, b+.026, .008), ".09 .17 .20 1", True)

    for j in range(23):
        x = -.75+j*.49
        for y in (-1.47, 1.47):
            box(world, f"aisle_{j}_{y}", (x, y, .003), (.17, .028, .003), ".77 .58 .20 1", True)
    for i, ((x, y), (tx, ty)) in enumerate(zip(STOPS, TARGETS), 1):
        node(world, "geom", name=f"paint_{i}", type="cylinder", pos=xyz((x, y, .005)),
             size=".28 .005", rgba=".10 .60 .58 .80", contype=0, conaffinity=0)
        node(world, "geom", name=f"marker_{i}", type="sphere", pos=xyz((tx, ty, .45)),
             size=.13, rgba=".98 .57 .13 1", contype=0, conaffinity=0)
        node(world, "site", name=f"target_{i}", pos=xyz((tx, ty, .45)), size=.012, rgba="1 .7 .05 1")
        for dx, dy, z in ((0, 0, .45), (0, .07, .5), (.06, 0, .42)):
            node(world, "geom", type="sphere", pos=xyz((tx+dx, ty+dy, z)),
                 size=.021, rgba=".14 .15 .16 1", contype=0, conaffinity=0)

    cart = node(world, "body", name="service_cart", mocap="true", pos=f"{CART_X} 2.48 0")
    box(cart, "cart", (0, 0, .36), (*CART_HALF, .36), ".86 .36 .09 1")
    box(cart, "cart_top", (0, 0, .74), (.55, .6, .04), ".20 .23 .25 1", True)
    for offset in np.linspace(-.43,.43,6):
        box(cart, f"cart_stripe_{offset}", (offset, -.575, .52), (.035, .009, .075), ".96 .74 .28 1", True)
    # Camera attached to the original chassis. +X is its forward axis.
    model = mj.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    chassis = model.body("base_link") or model.body("base")
    chassis.add_camera(name="inspection", pos=[.257, 0, .047], xyaxes=[0, -1, 0, 0, 0, 1], fovy=56)
    return model.compile()


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Go2Controller:
    def __init__(self, model, policy_path):
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.network = ort.InferenceSession(str(policy_path), options, providers=["CPUExecutionProvider"])
        self.input = self.network.get_inputs()[0].name
        joint_names = [f"{leg}_{joint}" for leg in ROBOT_LEGS for joint in ("hip", "thigh", "calf")]
        self.actuators = np.array([model.actuator(name).id for name in joint_names])
        joint_ids = model.actuator_trnid[self.actuators, 0]
        self.qpos = model.jnt_qposadr[joint_ids]
        self.qvel = model.jnt_dofadr[joint_ids]
        self.action = np.zeros(12)
        self.targets = HOME.copy()
        self.base = model.cam_bodyid[model.camera("inspection").id]
        self.integral = np.zeros(3)

    def command(self, model, data, cmd, hold=False):
        if hold:
            self.action.fill(0)
            self.integral.fill(0)
            self.targets = HOME.copy()
            return
        velocity = np.zeros(6)
        mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, self.base, velocity, 1)
        gravity = -data.xmat[self.base].reshape(3, 3)[2]
        measured = velocity[[3, 4, 2]]
        error = cmd - measured
        raw = cmd + .25 * error + self.integral
        update = (abs(raw) < .5) | (raw * error < 0)
        self.integral[update] = np.clip(self.integral[update] + 0.45 * error[update] * CONTROL_DT, -.25, .25)
        actor_cmd = np.clip(cmd + .25 * error + self.integral, [-.35,-.30,-.35], [.45,.30,.35])
        obs = np.concatenate([velocity[:3] * .25, gravity, actor_cmd,
                              data.qpos[self.qpos] - HOME, data.qvel[self.qvel] * .05, self.action])
        self.action = self.network.run(None, {self.input: obs.astype(np.float32)[None, :]})[0].ravel()
        self.targets = HOME + ACTION_SCALE * self.action

    def apply(self, model, data, hold=False):
        effort = (80 if hold else 20) * (self.targets - data.qpos[self.qpos])
        effort -= (3 if hold else .5) * data.qvel[self.qvel]
        limits = model.actuator_ctrlrange[self.actuators]
        data.ctrl[self.actuators] = np.clip(effort, np.maximum(limits[:, 0], -23.5), np.minimum(limits[:, 1], 23.5))


def path_target(path, xy, index):
    # Monotonic look-ahead with no jump across later portions of the route.
    while index < len(path) - 1 and np.linalg.norm(path[index] - xy) < .44:
        index += 1
    return path[index] - xy, index


def sight(model, data, number):
    idx = model.camera("inspection").id
    cam = data.cam_xpos[idx].copy()
    target = data.site(f"target_{number}").xpos
    delta = target - cam
    local = data.cam_xmat[idx].reshape(3, 3).T @ delta
    # The rendered RGB inset is 16:9, and the marker should be safely in frame.
    centered = -local[2] > 0 and abs(local[0] / local[2]) < .32 and abs(local[1] / local[2]) < .26
    geom = np.zeros(1, dtype=np.int32)
    dist = np.linalg.norm(delta)
    hit = mj.mj_ray(model, data, cam, delta/dist, np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8), 1, model.cam_bodyid[model.camera("inspection").id], geom)
    return bool(centered and (hit < 0 or hit >= dist - .16))


def run(args):
    model = build_scene(args.robot_xml)
    data = mj.MjData(model)
    controller = Go2Controller(model, args.policy)
    grid_path = load_planner(args.field_planner)
    data.qpos[:3] = [*START, .365]
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[controller.qpos] = HOME
    mj.mj_forward(model, data)
    path = plan(grid_path, START, STOPS[0], 2.48)
    plans = [{"reason": "initial", "t_s": 0, "xy": path.tolist()}]
    events, frames, camera_states = [], [], []
    step_index, stop, stable, obstacles, max_tilt = 0, 0, 0, 0, 0.
    foot_samples = {leg: [0, 0] for leg in ROBOT_LEGS}
    contacts = {model.geom(leg).id: leg for leg in ROBOT_LEGS}
    floor = model.geom("floor").id
    forbidden = {model.geom("cart").id, *(model.geom(f"fixture_{i}").id for i in range(len(FIXTURES)))}
    last_walk_xy = None
    walking_distance = 0.
    non_foot_floor_contacts = Counter()
    capture_end = -.1
    phase = "stand"
    hold = True
    parking = False
    setpoint = np.zeros(3)
    control_stride = round(CONTROL_DT / SIM_DT)
    steps = 0
    for k in range(round(args.duration / SIM_DT)):
        t = k * SIM_DT
        xy = data.qpos[:2].copy()
        rotation = data.xmat[controller.base].reshape(3, 3)
        heading = math.atan2(rotation[1, 0], rotation[0, 0])
        tilt = math.acos(float(np.clip(rotation[2, 2], -1, 1)))
        max_tilt = max(max_tilt, tilt)
        if k % control_stride == 0:
            if t >= 4.0 and len(plans) == 1:
                path = plan(grid_path, xy, STOPS[stop], 0.)
                old_path = np.array(plans[0]["xy"])
                crosses = np.any((abs(old_path[:,0]-CART_X) < CART_HALF[0]) & (abs(old_path[:,1]) < CART_HALF[1]))
                if not crosses:
                    raise RuntimeError("The cart must interrupt the original center route")
                plans.append({"reason": "cart crossing reported", "t_s": t, "xy": path.tolist()})
                events.append({"event": "replan", "t_s": t})
                step_index = 0
            data.mocap_pos[0, 1] = 2.48 * (1 - np.clip((t-4)/1.6, 0, 1))
            setpoint[:] = 0
            hold = t < .75 or t < capture_end or stop == len(STOPS)
            if stop == len(STOPS):
                phase = "complete"
                if t > capture_end + .80:
                    break
            elif t < .75:
                phase = "stand"
            elif t < capture_end:
                phase = "capture"
            else:
                goal = path[-1]
                error = np.linalg.norm(goal - xy)
                delta, step_index = path_target(path, xy, step_index)
                target_yaw = math.atan2(TARGETS[stop][1]-xy[1], TARGETS[stop][0]-xy[0])
                if error > (.34 if parking else .26):
                    parking = False
                    phase = "follow"
                    yaw = 0.0 if error > .70 else target_yaw
                    body_delta = rotation[:2, :2].T @ (delta if error > .70 else goal - xy)
                    setpoint[:2] = np.clip(1.25 * body_delta, [-.18, -.22], [.31, .22])
                    stable = 0
                else:
                    phase = "align"
                    yaw = target_yaw
                    velocity = np.linalg.norm(data.qvel[:2])
                    aligned = abs(wrap(yaw-heading)) <= math.radians(13)
                    if aligned and not parking:
                        parking = True
                    if parking and abs(wrap(yaw-heading)) > math.radians(18):
                        parking = False
                    hold = parking
                    good = (hold and velocity < .12 and abs(data.qvel[5]) < .15
                            and abs(wrap(yaw-heading)) < math.radians(18)
                            and sight(model, data, stop+1))
                    stable = stable + 1 if good else 0
                    if stable >= round(.40 / CONTROL_DT):
                        events.append({"event": "capture", "stop": stop+1, "t_s": t,
                                       "position_error_m": float(np.linalg.norm(np.array(STOPS[stop])-xy)),
                                       "heading_error_deg": abs(math.degrees(wrap(yaw-heading))),
                                       "speed_m_s": float(velocity), "target_visible": True})
                        camera_states.append(data.qpos.copy())
                        stop += 1
                        capture_end = t + 1.20
                        phase = "capture"
                        stable = 0
                        parking = False
                        if stop < len(STOPS):
                            path = plan(grid_path, xy, STOPS[stop], 0.)
                            plans.append({"reason": "next station", "t_s": capture_end, "xy": path.tolist()})
                            step_index = 0
                if not hold:
                    setpoint[2] = np.clip(1.3*wrap(yaw-heading), -.32, .32)
            controller.command(model, data, setpoint, hold)
        controller.apply(model, data, hold)
        mj.mj_step(model, data)
        steps += 1
        active_feet = set()
        for contact in data.contact:
            pair = (contact.geom1, contact.geom2)
            if floor in pair:
                other = pair[1] if pair[0] == floor else pair[0]
                if other in contacts:
                    active_feet.add(contacts[other])
                elif model.geom_bodyid[other] not in (0, model.body("service_cart").id):
                    non_foot_floor_contacts[model.body(model.geom_bodyid[other]).name] += 1
            elif forbidden.intersection(pair) and any(model.geom_bodyid[g] not in (0, model.body("service_cart").id) for g in pair):
                obstacles += 1
        if phase == "follow":
            for leg, count in foot_samples.items():
                count[int(leg in active_feet)] += 1
            if last_walk_xy is not None:
                walking_distance += float(np.linalg.norm(xy - last_walk_xy))
            last_walk_xy = xy
        else:
            last_walk_xy = None
        if k % 8 == 0:
            frames.append((t, data.qpos.copy(), data.mocap_pos.copy(), phase,
                           sum(data.contact[i].dist < 0 for i in range(data.ncon))))
        if data.qpos[2] < .14 or tilt > .75 or not np.isfinite(data.qpos).all():
            events.append({"event": "fall", "t_s": t})
            break
    result = {"mujoco": mj.__version__, "physics_dt_s": SIM_DT, "policy_dt_s": CONTROL_DT,
              "model_xml_sha256": digest(args.robot_xml),
              "planner_module_sha256": digest(args.field_planner),
              "policy_sha256": digest(args.policy), 
              "physics_steps": steps, "simulation_s": steps*SIM_DT,
              "position_m": data.qpos[:3].tolist(), "phase": phase, "max_body_tilt_deg": math.degrees(max_tilt),
              "obstacle_contact_samples": obstacles, "non_foot_floor_contact_samples_by_body": dict(non_foot_floor_contacts), "foot_samples_air_ground": foot_samples,
              "events": events, "plans": plans,
              "planar_distance_during_follow_m": walking_distance}
    result["completed_capture_sequence"] = (
        stop == len(STOPS) and obstacles == 0 and len(plans) == 3
        and all(min(count) > 0 for count in foot_samples.values())
        and not any(event["event"] == "fall" for event in events)
    )
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/"mission.json").write_text(json.dumps(result, indent=2) + "\n")
    np.savez_compressed(args.output/"state.npz", times=[f[0] for f in frames],
                        qpos=[f[1] for f in frames], mocap=[f[2] for f in frames],
                        phase=[f[3] for f in frames], contact=[f[4] for f in frames], capture_qpos=camera_states)
    print(json.dumps({k: v for k,v in result.items() if k != "plans"}, indent=2))
    return result["completed_capture_sequence"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-xml", required=True, type=Path)
    parser.add_argument("--policy", required=True, type=Path)
    parser.add_argument("--field-planner", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=85)
    raise SystemExit(0 if run(parser.parse_args()) else 1)


if __name__ == "__main__":
    main()
