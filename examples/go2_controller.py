"""Go2 joint torque policy adapter with chassis-frame angular velocity."""

from pathlib import Path
import mujoco as mj
import numpy as np
import onnxruntime as ort

ROBOT_LEGS = ("FR", "FL", "RR", "RL")
HOME = np.tile([0.0, 0.8, -1.5], 4)
ACTION_SCALE = np.tile([0.125, 0.25, 0.25], 4)
SIM_DT, CONTROL_DT = 0.002, 0.02


class Go2Controller:
    def __init__(self, model, policy_path):
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.network = ort.InferenceSession(
            str(policy_path), options, providers=["CPUExecutionProvider"]
        )
        self.input = self.network.get_inputs()[0].name
        joint_names = [
            f"{leg}_{joint}" for leg in ROBOT_LEGS for joint in ("hip", "thigh", "calf")
        ]
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
        gravity = -data.xmat[self.base].reshape(3, 3)[2]
        # Free-joint angular velocity is in the chassis frame. BODY-local
        # mj_objectVelocity uses the inertial frame, which differs on this model.
        measured = np.r_[
            (data.xmat[self.base].reshape(3, 3).T @ data.qvel[:3])[:2], data.qvel[5]
        ]
        error = cmd - measured
        raw = cmd + 0.8 * error + self.integral
        update = (abs(raw) < 0.5) | (raw * error < 0)
        self.integral[update] = np.clip(
            self.integral[update] + 1.5 * CONTROL_DT * error[update], -0.25, 0.25
        )
        actor_cmd = np.clip(cmd + 0.8 * error + self.integral, -0.5, 0.5)
        obs = np.concatenate(
            [
                data.qvel[3:6] * 0.25,
                gravity,
                actor_cmd,
                data.qpos[self.qpos] - HOME,
                data.qvel[self.qvel] * 0.05,
                self.action,
            ]
        )
        self.action = self.network.run(
            None, {self.input: obs.astype(np.float32)[None, :]}
        )[0].ravel()
        self.targets = HOME + ACTION_SCALE * self.action

    def apply(self, model, data, hold=False):
        effort = (80 if hold else 20) * (self.targets - data.qpos[self.qpos])
        effort -= (3 if hold else 0.5) * data.qvel[self.qvel]
        limits = model.actuator_ctrlrange[self.actuators]
        data.ctrl[self.actuators] = np.clip(
            effort, np.maximum(limits[:, 0], -23.5), np.minimum(limits[:, 1], 23.5)
        )
