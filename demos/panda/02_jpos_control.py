from mujoco_py import MjSim, MjViewer
import numpy as np
import os, sys

sim_path = os.path.dirname(os.path.dirname(os.getcwd()))
sys.path.append(sim_path)

# If not installed pykin library, append pykin to sys.path
pykin_path = sim_path+"/pykin/"
sys.path.append(pykin_path)

from models.world import MujocoWorldBase
from models.robots import Panda
from models.arenas import BinsArena
from models.grippers import PandaGripper
from controllers.joint_pos import JointPositionController
from utils.common import load_pykin, get_result_qpos

def main():
    world = MujocoWorldBase()

    mujoco_robot = Panda()
    mujoco_robot.set_base_xpos([0.0, 0, 0.913])

    mujoco_arena = BinsArena()
    mujoco_arena.set_origin([0.8, 0, 0])

    gripper = PandaGripper()
    mujoco_robot.add_gripper("panda_right_hand", gripper)

    world.merge(mujoco_robot)
    world.merge(mujoco_arena)

    model = world.get_model(mode="mujoco_py")
    sim = MjSim(model)
    viewer = MjViewer(sim)
    viewer.vopt.geomgroup[0] = 0

    panda_robot = load_pykin(sim_path + '/pykin/asset/urdf/panda/panda.urdf')
    panda_robot.setup_link_name("panda_link_0", "panda_right_hand")

    init_qpos = [0, 0.1963495375, 0.00, -2.616, 0.00, 2.9415926, 0.78539815]
    desired_qpos = np.array([0, 0, 0, -1.5708, 0, 1.8675, 0])
    transformations = panda_robot.forward_kin(desired_qpos)
    eef_pose = transformations[panda_robot.eef_name].pose
    result_qpos = get_result_qpos(panda_robot, init_qpos, eef_pose)

    jpos_controller = JointPositionController(sim=sim, eef_name=panda_robot.eef_name)
    jpos_controller.kp = 20
    
    while True:
        torque = jpos_controller.run_controller(sim, result_qpos)
        sim.data.ctrl[jpos_controller.qpos_index] = torque
        sim.data.ctrl[jpos_controller.gripper_index] = [0.04, -0.04]

        sim.step()
        viewer.render()

if __name__ == "__main__":
    main()