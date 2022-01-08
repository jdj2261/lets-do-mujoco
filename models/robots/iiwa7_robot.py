import numpy as np
from models.robots.robot import MujocoRobot
from utils.mjcf_utils import xml_path_completion


class IIWA7(MujocoRobot):
    """
    IIWA is a bright and spunky robot created by KUKA
    """
    
    def __init__(
        self, 
        pos=[0, 0, 0.913], 
        rot=[0, 0, 0], 
        xml_path="robots/iiwa7/robot.xml"
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
        return ["iiwa7_link_{}".format(x) for x in range(1, 8)]
        
    @property
    def joints(self):
        return ["iiwa7_joint_{}".format(x) for x in range(1, 8)]

    @property
    def actuators(self):
        return ["iiwa7_torq_j{}".format(x) for x in range(1,8)]

    @property
    def contact_geoms(self):
        return ["iiwa7_link_{}_collision".format(x) for x in range(8)]

    @property
    def visual_geoms(self):
        return ["iiwa7_link_{}_visual".format(x) for x in range(8)]

    @property
    def init_qpos(self):
        return np.array([0.000, 0.650, 0.000, -1.890, 0.000, 0.600, 0.000])

    @property
    def base_name(self):
        return "iiwa7_link_0"

    @property
    def eef_name(self):
        return "iiwa7_right_hand"
