"""
6-DoF gripper with its open/close variant
"""
import numpy as np
from utils.mjcf_utils import xml_path_completion
from models.grippers.gripper import Gripper


class Robotiq85GripperBase(Gripper):
    """
    6-DoF Robotiq gripper.
    """

    def __init__(self):
        super().__init__(xml_path_completion("grippers/robotiq_gripper_85.xml"))

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return [3.3161, 0., 0., 0., 0., 0.]

    @property
    def joints(self):
        return ["finger_joint", "left_inner_finger_joint",
                "left_inner_knuckle_joint", "right_outer_knuckle_joint",
                "right_inner_finger_joint", "right_inner_knuckle_joint"]

    @property
    def actuators(self):
        return ["finger_joint", "right_outer_knuckle_joint"]

    @property
    def dof(self):
        return 6

    @property
    def visualization_sites(self):
        return ["grip_site", "grip_site_cylinder"]

    @property
    def contact_geoms(self):
        return [
            "hand_collision",
            "left_outer_knuckle_collision",
            "left_outer_finger_collision",
            "left_inner_finger_collision",
            "left_fingertip_collision",
            "left_inner_knuckle_collision",
            "right_outer_knuckle_collision",
            "right_outer_finger_collision",
            "right_inner_finger_collision",
            "right_fingertip_collision",
            "right_inner_knuckle_collision",
        ]

    @property
    def left_finger_geoms(self):
        return [
            "left_outer_finger_collision",
            "left_inner_finger_collision",
            "left_fingertip_collision"
        ]

    @property
    def right_finger_geoms(self):
        return [
            "right_outer_finger_collision",
            "right_inner_finger_collision",
            "right_fingertip_collision"
        ]

class Robotiq85Gripper(Robotiq85GripperBase):
    """
    1-DoF variant of RobotiqGripperBase.
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
        self.current_action = np.clip(self.current_action + self.speed * np.sign(action), -1.0, 1.0)
        return self.current_action

    @property
    def speed(self):
        return 0.01

    @property
    def dof(self):
        return 1
