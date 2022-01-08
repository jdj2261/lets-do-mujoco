import numpy as np
from utils.transform_utils import get_rpy_from_quaternion

class PicknPlace:
    def __init__(
        self, 
        sim,
        viewer,
        mujoco_robot,
        pykin_robot,
        objects,
        controller,
        gripper=None,
        cartesisan_planner=None,
        rrt_planner=None,
    ):
        self._sim = sim
        self._viewer = viewer
        self._mj_robot = mujoco_robot
        self._pykin_robot = pykin_robot
        self._objects = objects
        self._controller = controller
        self._gripper = gripper
        self.has_gripper = gripper is not None
        self._c_planner = cartesisan_planner
        self._rrt_planner = rrt_planner

        self._get_references()
        self._place_poses = None

    def _get_references(self):
        """
        Sets up necessary reference for robots, grippers, and objects.
        """
        self._get_robot_ref()
        self._get_object_ref()
        self._get_gripper_ref()
        self._get_actuator_ref()

    def _get_robot_ref(self):
        self.robot_joints = list(self._mj_robot.joints)
        self._ref_joint_pos_indexes = [
            self._sim.model.get_joint_qpos_addr(x) for x in self.robot_joints
        ]
        self._ref_joint_vel_indexes = [
            self._sim.model.get_joint_qvel_addr(x) for x in self.robot_joints
        ]

    def _get_object_ref(self):
        self.obj_body_id = {}
        self.obj_geom_id = {}

        self.l_finger_geom_ids = [
            self._sim.model.geom_name2id(x) for x in self._gripper.left_finger_geoms
        ]
        self.r_finger_geom_ids = [
            self._sim.model.geom_name2id(x) for x in self._gripper.right_finger_geoms
        ]

        for obj in self._objects:
            obj_name = obj.name + "_col"
            geom_name = obj_name+ "-0"
            self.obj_body_id[obj_name] = self._sim.model.body_name2id(obj_name)
            self.obj_geom_id[obj_name] = self._sim.model.geom_name2id(geom_name)

    def _get_gripper_ref(self):
        if self.has_gripper:
            self.gripper_joints = self._gripper.joints
            self._ref_gripper_joint_pos_indexes = [
                self._sim.model.get_joint_qpos_addr(x) for x in self.gripper_joints
            ]
            self._ref_gripper_joint_vel_indexes = [
                self._sim.model.get_joint_qvel_addr(x) for x in self.gripper_joints
            ]

        self.eef_site_id = self._sim.model.site_name2id(self._gripper.visualization_sites[0])
        self.eef_cylinder_id = self._sim.model.site_name2id(self._gripper.visualization_sites[1])

    def _get_actuator_ref(self):
        self._ref_joint_torq_actuator_indexes = [
            self._sim.model.actuator_name2id(actuator)
            for actuator in self._sim.model.actuator_names
            if "torq" in actuator
        ]

        if self.has_gripper:
            self._ref_joint_gripper_actuator_indexes = [
                self._sim.model.actuator_name2id(actuator)
                for actuator in self._sim.model.actuator_names
                if actuator.startswith("gripper")
            ]

    def pick(self, pose=None):
        if pose is not None:
            self.jmove_in_cspace(pose)
            self.open_gripper()
            self.close_gripper()
        else:
            for obj in self._objects:
                obj_name = obj.name + "_col"
                grasp_pose = self.compute_grasp_pose(obj_name)
                self.jmove_in_cspace(np.hstack((grasp_pose, np.array([-2.50435769e-02,
                    9.21807347e-01,  3.82091147e-01, -6.04184456e-02]))))
                self.open_gripper()
                self.cmove([ 0.7, -0.3, 0.97])
                self.close_gripper()
        # self.jmove_in_cspace(pose)
        # self.open_gripper()
        # self.cmove([ 0.7, -0.3, 0.97])
        # self.close_gripper()
            
    def place(self, pose=None):
        if pose is not None:
            self.jmove_in_cspace(pose)
            self.open_gripper()
            self.close_gripper()
        else:
            if self._place_poses is None:
                raise TypeError("Check the place positions")

            for pos in self._place_poses:
                self.jmove_in_cspace(np.hstack((pos, np.array([-2.50435769e-02,
                    9.21807347e-01,  3.82091147e-01, -6.04184456e-02]))))
                self.cmove([0.6, 0.4, 1.0])
                self.open_gripper()

    def set_place_area(self, poses):
        poses = [np.array(poses)]
        if len(poses) >= 1:
            poses = [np.array(pose).reshape(3,) for pose in poses]
        self._place_poses = poses

    def move_to_init_pose(self):
        self.jmove_in_jspace(self._mj_robot.init_qpos)

    def compute_grasp_pose(self, obj_name):
        return self.object_pos(obj_name) + np.array([0, 0, 0.15])

    def _check_ik_solution(self, init_qpos, target_pose):
        is_limit_qpos = False

        target_pose = np.array(target_pose)
        if target_pose.shape == (3,):
            target_pose = np.hstack((target_pose, np.array([1, 0, 0, 0])))

        result_qpos = self._pykin_robot.inverse_kin(init_qpos, target_pose, method="LM")
        is_limit_qpos = self._pykin_robot.check_limit_joint(result_qpos)
        if is_limit_qpos:
            return result_qpos

        while not is_limit_qpos:
            result_qpos = self._pykin_robot.inverse_kin(np.random.randn(len(init_qpos)), target_pose, method="LM")
            is_limit_qpos = self._pykin_robot.check_limit_joint(result_qpos)
        return result_qpos

    def approach_pose_using_rrt(self, pose, is_interpolated=True):
        interpolated_path, joint_path = self._rrt_planner.get_path_in_joinst_space(
            cur_q=self._controller.q_pos, 
            goal_pose=pose,
            resolution=1)
        
        if is_interpolated:
            for _, qpos in enumerate(interpolated_path):
                self.jmove_in_jspace(qpos)
        else:
            for _, qpos in enumerate(joint_path):
                self.jmove_in_jspace(qpos)
        print("reach")

    def jmove_in_cspace(self, pose):
        init_qpos = self._mj_robot.init_qpos
        result_qpos = self._check_ik_solution(init_qpos, pose)
        self.jmove_in_jspace(result_qpos)

    def jmove_in_jspace(self, joints):
        while not self._controller.is_reached():
            self.torque = self._controller.run_controller(self._sim, joints)
            self._sim.data.ctrl[self._controller.qpos_index] = self.torque
            self._sim.step()
            self._viewer.render()

    def cmove(self, pose, resolution=0.01, damping=0.03, pos_sensitivity=0.03, is_slerp=False):
        joint_path, _ = self._c_planner.get_path_in_joinst_space(            
            current_q=self._controller.q_pos,
            goal_pose=pose,
            resolution=resolution, 
            damping=damping,
            pos_sensitivity=pos_sensitivity,
            is_slerp=is_slerp)

        for _, qpos in enumerate(joint_path):
            self.jmove_in_jspace(qpos)
        print("reach")

    def stop(self):
        pass

    def open_gripper(self):
        for _ in range(1000):
            torqe = self._controller.run_controller(self._sim, self._controller.q_pos)
            self._sim.data.ctrl[self._controller.qpos_index] = torqe
            self._sim.data.ctrl[self._controller.gripper_index] = [0.4, -0.4]
            # self._sim.data.ctrl[self._controller.gripper_index] = [-0.020833, 0.020833] // Rethink
            # self._sim.data.ctrl[self._controller.gripper_index] = [0.0, 0.0] // Robotiq140
            self._sim.step()
            self._viewer.render()

    def close_gripper(self):
        for _ in range(1000):
            torqe = self._controller.run_controller(self._sim, self._controller.q_pos)
            self._sim.data.ctrl[self._controller.qpos_index] = torqe
            self._sim.data.ctrl[self._controller.gripper_index] = [0.0, 0.0]
            # self._sim.data.ctrl[self._controller.gripper_index] = [0.7, -0.7] // Robotiq140
            # self._sim.data.ctrl[self._controller.gripper_index] = [0.0115, -0.0115] // Rethink
            self._sim.step()
            self._viewer.render()

    def _check_grasp_pose(self, obj_name):
        gripper_site_pos = self.sim.data.site_xpos[self.eef_site_id]
        obj_pos = self.sim.data.body_xpos[self.obj_body_id[obj_name]]
        dist = np.linalg.norm(gripper_site_pos - obj_pos)

        return dist < 0.15

    def _check_grasp(self):
        pass

    @property
    def griper_pose(self):
        return self._controller.eef_pose

    def object_pos(self, object_name):
        return np.array(self._sim.data.body_xpos[self.obj_body_id[object_name]])