import numpy as np
from models.robots.robot import MujocoRobot
from utils.mjcf_utils import xml_path_completion


class UR5e(MujocoRobot):
    """Panda is a sensitive single-arm robot designed by Franka."""
    def __init__(
        self, 
        pos=[0, 0, 0.913], 
        rot=[0, 0, 0], 
        xml_path="robots/ur5e/robot.xml"
    ):
        super().__init__(xml_path_completion(xml_path))
        self._setup_base_pose(pos, rot)

    @property
    def bottom_offset(self):
        return np.array([0, 0, 0])

    @property
    def dof(self):
        return 6

    @property
    def bodies(self):
        return ["ur5e_shoulder_link", "ur5e_upper_arm_link", "ur5e_forearm_link",\
                "ur5e_wrist_1_link", "ur5e_wrist_2_link", "ur5e_wrist_3_link"]
    @property
    def joints(self):
        return ["ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint",\
                "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"]

    @property
    def actuators(self):
        return ["ur5e_torq_j{}".format(x) for x in range(1, 8)]

    @property
    def contact_geoms(self):
        return ["ur5e_base_col", "ur5e_upperarm_col", "ur5e_forearm_col",\
                "ur5e_wrist1_col", "ur5e_wrist2_col", "ur5e_wrist3_col"]

    @property
    def visual_geoms(self):
        return ["ur5e_base_vis", "ur5e_upperarm_vis", "ur5e_forearm_vis",\
                "ur5e_wrist1_vis", "ur5e_wrist2_vis", "ur5e_wrist3_vis"]

    @property
    def init_qpos(self):
        return np.array([-0.470, -1.735, 2.480, -2.275, -1.590, -1.991])

    @property
    def base_name(self):
        return 'ur5e_base_link'

    @property
    def eef_name(self):
        return "ur5e_right_hand"


