from collections import OrderedDict
from models.base import MujocoXML
from utils.mjcf_utils import array_to_string
from utils.transform_utils import euler2mat, mat2quat

class MujocoRobot(MujocoXML):
    """Base class for all robot models."""

    def __init__(self, fname):
        """Initializes from file @fname."""
        super().__init__(fname)

        self.grippers = OrderedDict()

    def set_base_xpos(self, pos):
        self.base_body.set("pos", array_to_string(pos - self.bottom_offset))

    def set_base_ori(self, rot):
        # xml quat assumes w,x,y,z so we need to convert to this format from outputted x,y,z,w format from fcn
        rot = mat2quat(euler2mat(rot))[[3,0,1,2]]
        self.base_body.set("quat", array_to_string(rot))

    def add_gripper(self, arm_name, gripper):
        if arm_name in self.grippers:
            raise ValueError("Attempts to add multiple grippers to one body")

        arm_subtree = self.worldbody.find(".//body[@name='{}']".format(arm_name))

        for body in gripper.worldbody:
            arm_subtree.append(body)
        
        self.merge(gripper, merge_body=False)
        self.grippers[arm_name] = gripper

    def _setup_base_pose(self, pos, rot):
        self.base_body.set("pos", array_to_string(pos - self.bottom_offset))
        rot = mat2quat(euler2mat(rot))[[3,0,1,2]]
        self.base_body.set("quat", array_to_string(rot))

    @property
    def root_body(self):
        return self.name

    @property
    def base_body(self):
        node = self.worldbody.find("./body[@name='{}']".format(self.base_name))
        return node

    @property
    def bottom_offset(self):
        raise NotImplementedError

    @property
    def dof(self):
        raise NotImplementedError

    @property
    def bodies(self):
        raise NotImplementedError

    @property
    def joints(self):
        raise NotImplementedError

    @property
    def actuators(self):
        raise NotImplementedError

    @property
    def contact_geoms(self):
        raise NotImplementedError

    @property
    def visual_geoms(self):
        raise NotImplementedError
    
    @property
    def init_qpos(self):
        raise NotImplementedError

    @property
    def base_name(self):
        raise NotImplementedError

    @property
    def eef_name(self):
        raise NotImplementedError