"""
Defines the base class of all grippers
"""
import numpy as np
from models.base import MujocoXML

class Gripper(MujocoXML):
    """
    Base class for grippers

    Args:
        fname (str): Path to relevant xml file to create this gripper instance
    """

    def __init__(self, fname):
        super().__init__(fname)

        # Set variable to hold current action being outputted
        self.current_action = np.zeros(self.dof)

        # Grab gripper offset (string -> np.array -> elements [1, 2, 3, 0] (x, y, z, w))
        self.rotation_offset = np.fromstring(self.worldbody[0].attrib.get("quat", "1 0 0 0"),
                                             dtype=np.float64, sep=" ")[[1, 2, 3, 0]]

    def format_action(self, action):
        """
        Given (-1,1) abstract control as np-array
        returns the (-1,1) control signals
        for underlying actuators as 1-d np array
        """
        raise NotImplementedError

    @property
    def init_qpos(self):
        """
        Returns rest(open) qpos of the gripper
        """
        raise NotImplementedError

    @property
    def dof(self):
        """
        Returns the number of DOF of the gripper
        """
        raise NotImplementedError

    @property
    def joints(self):
        """
        Returns a list of joint names of the gripper
        """
        raise NotImplementedError

    @property
    def actuators(self):
        """
        Returns a list of actuators names of the gripper
        """
        raise NotImplementedError

    @property
    def contact_geoms(self):
        """
        Returns a list of names corresponding to the geoms
        used to determine contact with the gripper.
        """
        return []

    @property
    def visualization_sites(self):
        """
        Returns a list of sites corresponding to the geoms
        used to aid visualization by human.
        (and should be hidden from robots)
        """
        return []

    @property
    def visualization_geoms(self):
        """
        Returns a list of sites corresponding to the geoms
        used to aid visualization by human.
        (and should be hidden from robots)
        """
        return []

    @property
    def left_finger_geoms(self):
        """
        Geoms corresponding to left finger of a gripper
        """
        raise NotImplementedError

    @property
    def right_finger_geoms(self):
        """
        Geoms corresponding to raise finger of a gripper
        """
        raise NotImplementedError
