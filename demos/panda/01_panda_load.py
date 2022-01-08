from mujoco_py import MjSim, MjViewer
import numpy as np
import os, sys

sim_path = os.path.dirname(os.path.dirname(os.getcwd()))
sys.path.append(sim_path)

from models.world import MujocoWorldBase
from models.robots import Panda
from models.arenas import BinsArena
from models.objects import CanObject
from models.grippers import PandaGripper

def main():
    world = MujocoWorldBase()

    mujoco_robot = Panda()
    mujoco_robot.set_base_xpos([0.0, 0, 0.913])

    mujoco_arena = BinsArena()
    mujoco_arena.set_origin([0.6, 0, 0])

    gripper = PandaGripper()
    mujoco_robot.add_gripper("panda_right_hand", gripper)

    mujoco_object = CanObject(pos=[0.7, -0.3, 0.87])

    world.merge(mujoco_robot)
    world.merge(mujoco_arena)
    world.merge(mujoco_object, object_type="object")

    model = world.get_model(mode="mujoco_py")
    sim = MjSim(model)
    viewer = MjViewer(sim)
    viewer.vopt.geomgroup[0] = 0

    while True:
        sim.data.ctrl[:7] = np.random.randn(7)
        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()

