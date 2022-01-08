import numpy as np
from models.robots.robot import MujocoRobot
from utils.mjcf_utils import xml_path_completion


class Panda(MujocoRobot):
    """Panda is a sensitive single-arm robot designed by Franka."""
    def __init__(
        self, 
        pos=[0, 0, 0.913], 
        rot=[0, 0, 0], 
        xml_path="robots/panda/robot.xml"
    ):
        super().__init__(xml_path_completion(xml_path))
        self._setup_base_pose(pos, rot)

    @property
    def bottom_offset(self):
        return np.array([0, 0, 0])

    @property
    def dof(self):
        return 7

    @property
    def bodies(self):
        return ["panda_link_{}".format(x) for x in range(1, 8)]
        
    @property
    def joints(self):
        return ["panda_joint_{}".format(x) for x in range(1, 8)]

    @property
    def actuators(self):
        return ["panda_torq_j{}".format(x) for x in range(1, 8)]

    @property
    def contact_geoms(self):
        return ["panda_link_{}_collision".format(x) for x in range(8)]

    @property
    def visual_geoms(self):
        return ["panda_link_{}_visual".format(x) for x in range(8)]

    @property
    def init_qpos(self):
        return np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])

    @property
    def base_name(self):
        return "panda_link_0"

    @property
    def eef_name(self):
        return "panda_right_hand"


