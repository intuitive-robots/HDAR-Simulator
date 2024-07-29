import mujoco
import mujoco.viewer
import numpy as np
from controllers.tcp_controller import InteractiveTCPControllerBase
import time
import matplotlib.pyplot as plt
from alr_sim.core import Scene, RobotBase

class MyTCPSensorPos(InteractiveTCPControllerBase):
    def __init__(self, Scene, robot: RobotBase, robot_config: dict):
        super().__init__(Scene, robot, robot_config)
        self.desired_pos = np.array(robot_config["init_end_eff_pos"])
        self.desired_quat = np.array(robot_config["init_end_eff_quat"])

    def read_ctrl_pos(self):
        # 实现确定期望位置的逻辑
        return self.desired_pos

    def read_ctrl_quat(self):
        # 实现确定期望方向的逻辑
        return self.desired_quat
    


TCPforce_sensordata = []