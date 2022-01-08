import copy
import xml.etree.ElementTree as ET
import numpy as np
from models.base import MujocoXML
from utils.mjcf_utils import string_to_array, array_to_string, new_joint, new_geom
from utils.transform_utils import euler2mat, mat2quat

class MujocoObject:

    def __init__(self):
        pass

    @property
    def base_xpos(self):
        raise NotImplementedError

    @property
    def base_ori(self):
        raise NotImplementedError

    def get_collision(self):
        raise NotImplementedError


class MujocoXMLObject(MujocoXML, MujocoObject):
    """
    MujocoObjects that are loaded from xml files
    """

    def __init__(self, fname, name, pos, rot, joints):
        MujocoXML.__init__(self, fname)

        self.name = name

        assert np.array(rot).shape == (3,), "Orientation type is Euler!!"
        rot = mat2quat(euler2mat(rot))[[3,0,1,2]]

        self._joints = joints

        self._base_object = self.worldbody.find("./body")
        self._bottom_site = self.worldbody.find("./body/site[@name='bottom_site']")
        self._top_site = self.worldbody.find("./body/site[@name='top_site']")
        self._horizontal_radius_site = self.worldbody.find("./body/site[@name='horizontal_radius_site']")
        
        self._base_object.set("pos", array_to_string(pos))
        self._base_object.set("quat", array_to_string(rot))

        self._base_size = None

    @property
    def bottom_offset(self):
        return string_to_array(self._bottom_site.get("pos"))

    @property
    def top_offset(self):
        return string_to_array(self._top_site.get("pos"))

    @property
    def horizontal_radius(self):
        return string_to_array(self._horizontal_radius_site.get("pos"))[0]

    @property
    def base_object(self):
        return self._base_object

    @base_object.setter
    def base_object(self, base_name):
        assert type(base_name) == str
        self._base_object = self.worldbody.find("{}".format(base_name))

    @property
    def base_xpos(self):
        return string_to_array(self._base_object.get("pos"))
        
    @base_xpos.setter
    def base_xpos(self, pos):
        self._base_object.set("pos", array_to_string(pos))

    @property
    def base_ori(self):
        return string_to_array(self._base_object.get("quat"))
        
    @base_ori.setter
    def base_ori(self, rot):
        assert np.array(rot) == (3,), "Orientation type is Euler!!"
        rot = mat2quat(euler2mat(rot))[[3,0,1,2]]
        self._base_object.set("quat", array_to_string(rot))

    @property
    def base_size(self):
        return string_to_array(self._base_object.get("size"))

    @base_size.setter
    def base_size(self, size):
        self._base_size = self._base_object.set("size", array_to_string(size))

    def get_collision(self):
        collision = copy.deepcopy(self.worldbody.find("./body/body[@name='collision']"))
        collision.attrib.pop("name")
        col_name = self.name+"_col"

        geoms = collision.findall("geom")
        duplicate_geoms = copy.deepcopy(geoms)
        if self.name is not None:
            collision.attrib["name"] = col_name
            if len(geoms) == 1:
                geoms[0].set("name", col_name+"-0")
            else:
                for i in range(len(geoms)):
                    geoms[i].set("name", "{}-{}".format(col_name, i))
        
        geom_group = duplicate_geoms[0].get("group")
        duplicate_geoms[0].set("group", "1")
        
        if int(geom_group) == 1:
            duplicate_geoms[0].set("group", "0")
        
        collision.append(ET.Element("geom", attrib=duplicate_geoms[0].attrib))
        
        collision.set("pos", array_to_string(self.base_xpos))
        collision.set("quat", array_to_string(self.base_ori))

        if self._joints is not None:
            collision.append(new_joint(name=col_name+"_joint", **self._joints[0]))
        return collision