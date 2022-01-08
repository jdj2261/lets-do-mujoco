"""
Gripper with two fingers for Rethink Robots.
"""
import numpy as np
from utils.mjcf_utils import xml_path_completion
from models.grippers.gripper import Gripper


class RethinkGripperBase(Gripper):
    """
    Gripper with long two-fingered parallel jaw.
    """

    def __init__(self):
        super().__init__(xml_path_completion("grippers/rethink_gripper.xml"))

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.020833, -0.020833])

    @property
    def joints(self):
        return ["r_finger_joint", "l_finger_joint"]

    @property
    def actuators(self):
        return ["gripper_r_finger_joint", "gripper_l_finger_joint"]

    @property
    def dof(self):
        return 2

    @property
    def visualization_sites(self):
        return ["grip_site", "grip_site_cylinder"]

    @property
    def contact_geoms(self):
        return [
            "r_finger_g0",
            "r_finger_g1",
            "l_finger_g0",
            "l_finger_g1",
            "r_fingertip_g0",
            "l_fingertip_g0",
        ]

    @property
    def left_finger_geoms(self):
        return ["l_finger_g0", "l_finger_g1", "l_fingertip_g0"]

    @property
    def right_finger_geoms(self):
        return ["r_finger_g0", "r_finger_g1", "r_fingertip_g0"]


class RethinkGripper(RethinkGripperBase):
    """
    Modifies two finger base to only take one action.
    """

    def format_action(self, action):
        """
        Maps continuous action into binary output
        -1 => open, 1 => closed

        Args:
            action (np.array): gripper-specific action

        Raises:
            AssertionError: [Invalid action dimension size]
        """
        assert len(action) == 1
        self.current_action = np.clip(self.current_action + np.array([1.0, -1.0]) * self.speed * np.sign(action), -1.0, 1.0)
        return self.current_action

    @property
    def speed(self):
        return 0.01

    @property
    def dof(self):
        return 1
