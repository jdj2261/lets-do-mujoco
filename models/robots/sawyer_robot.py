import numpy as np
from models.robots.robot import MujocoRobot
from utils.mjcf_utils import xml_path_completion


class Sawyer(MujocoRobot):
    """
    Sawyer is a witty single-arm robot designed by Rethink Robotics.
    """

    def __init__(
        self, 
        pos=[0, 0, 0.913], 
        rot=[0, 0, 0], 
        xml_path="robots/sawyer/robot.xml"
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
        return ["sawyer_link_{}".format(x) for x in range(1, 8)]
        
    @property
    def joints(self):
        return ["sawyer_joint_{}".format(x) for x in range(1, 8)]

    @property
    def actuators(self):
        return ["sawyer_torq_j{}".format(x) for x in range(1, 8)]

    @property
    def contact_geoms(self):
        return ["sawyer_link_{}_collision".format(x) for x in range(8)]

    @property
    def visual_geoms(self):
        return ["sawyer_link_{}_visual".format(x) for x in range(8)]

    @property
    def init_qpos(self):
        return np.array([0, 0, -1.18, 0.00, 2.18, 0.00, 0.57, -1.57])

    @property
    def base_name(self):
        return 'sawyer_base'

    @property
    def eef_name(self):
        return "sawyer_right_hand"
