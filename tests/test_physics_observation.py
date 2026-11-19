"""Guard the chassis/inertial frame mismatch in the optional physics example."""

import unittest
from types import SimpleNamespace

import numpy as np

try:
    import mujoco as mj
    from examples.go2_controller import Go2Controller, HOME
except ImportError:
    mj = None


@unittest.skipIf(
    mj is None, "The physics example requires optional MuJoCo dependencies"
)
class ChassisAngularVelocityTest(unittest.TestCase):
    def test_policy_reads_chassis_axes_with_rotated_inertia(self):
        model = mj.MjModel.from_xml_string("""
        <mujoco><worldbody><body name="robot">
          <freejoint/>
          <inertial pos="0 0 0" mass="1" diaginertia="1 2 3"
                    quat=".70710678 0 .70710678 0"/>
          <geom size=".1" mass="0"/>
        </body></worldbody></mujoco>
        """)
        data = mj.MjData(model)
        expected = np.array([0.1, 0.2, 0.3])
        data.qvel[3:6] = expected
        mj.mj_forward(model, data)
        inertial_velocity = np.zeros(6)
        mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, 1, inertial_velocity, 1)
        self.assertFalse(np.allclose(inertial_velocity[:3], expected))
        network = SimpleNamespace()

        def infer(_, inputs):
            network.observation = inputs["obs"]
            return [np.zeros((1, 12), dtype=np.float32)]

        network.run = infer
        controller = Go2Controller.__new__(Go2Controller)
        controller.network = network
        controller.input = "obs"
        controller.base = 1
        controller.qpos = np.zeros(12, dtype=int)
        controller.qvel = np.zeros(12, dtype=int)
        controller.action = np.zeros(12)
        controller.integral = np.zeros(3)
        controller.targets = HOME.copy()
        controller.command(model, data, np.zeros(3))
        np.testing.assert_allclose(
            network.observation[0, :3], expected * 0.25, atol=1e-7
        )


if __name__ == "__main__":
    unittest.main()
