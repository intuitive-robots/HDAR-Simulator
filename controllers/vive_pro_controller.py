import numpy as np

from alr_sim.core import RobotBase
from alr_sim.sims.mj_beta import MjScene
from alr_sim.utils import (
    euler2quat,
    euler2mat,
    quat2mat,
    quat_mul,
)

from .triad_openvr import triad_openvr
from .tcp_controller import InteractiveTCPControllerBase
from utils import degEuler2radEuler


class ViveProMotionControllerTCPController(InteractiveTCPControllerBase):
    def __init__(self, scene: MjScene, robot: RobotBase, robot_config: dict):
        super().__init__(scene, robot, robot_config)
        self.device = triad_openvr()
        self.device.print_discovered_objects()
        self.T_r: np.ndarray = np.eye(4)
        self.position = robot_config["init_end_eff_pos"]
        self.quat = robot_config["init_end_eff_quat"]
        self.resetOrigin(self.position, self.quat)

    def getControl(self, robot: RobotBase):
        state = self.device.devices["controller_1"].get_controller_inputs()
        if state["trigger"] <= 0.99:
            robot.open_fingers()
        else:
            robot.close_fingers(duration=0.0)
        self.updateMotionControllerPose()
        return super().getControl(robot)

    def updateMotionControllerPose(self):
        position, rotation = self.getMotionControllerPose()
        if position is None:
            return

        self.position = [-position[2] - 0.1, -position[0], position[1] + 0.0]
        rotation = [-rotation[0], -rotation[2] + 45, -rotation[1]]
        self.quat = quat_mul(
            np.array(euler2quat(degEuler2radEuler(rotation))), np.array([0, 1, 0, 0])
        )

    def resetOrigin(self, ef_pose=None, ef_quat=None):
        position, rotation = None, None
        while position is None:
            position, rotation = self.getMotionControllerPose()

        T_mc = np.eye(4)
        T_mc[:3, :3] = euler2mat(degEuler2radEuler(rotation))
        T_mc[:3, 3] = position

        self.T_ef = np.eye(4)
        if ef_pose is None:
            ef_pose = self.robot.current_c_pos_global
            ef_quat = self.robot.current_c_quat_global

        self.T_ef[:3, :3] = quat2mat([1, 0, 0, 0])
        self.T_ef[:3, 3] = ef_pose
        self.R_T = quat2mat(ef_quat)
        self.T_r = self.T_ef @ np.linalg.inv(T_mc)

    def getMotionControllerPose(self):
        pose = self.device.devices["controller_1"].get_pose_euler()
        if pose is None:
            return None, None
        return pose[:3], pose[3:]

    def read_ctrl_pos(self):
        return self.position

    def read_ctrl_quat(self):
        return self.quat