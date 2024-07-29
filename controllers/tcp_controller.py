import abc
from alr_sim.controllers.IKControllers import CartPosQuatImpedenceController
from alr_sim.core import RobotBase
from alr_sim.sims.mj_beta import MjScene
from alr_sim.sims.universal_sim.PrimitiveObjects import Box
import numpy as np
from pynput import keyboard
from threading import Thread


class InteractiveTCPControllerBase(CartPosQuatImpedenceController):
    def __init__(self, scene: MjScene, robot: RobotBase, robot_config: dict):
        super().__init__()
        self.scene: MjScene = scene
        self.robot: RobotBase = robot
        self.indicator_name = getattr(robot, "name") + "_tcp_indicator"
        self.robot_config = robot_config
        self.scene.add_object(
             Box(
                 self.indicator_name,
                 robot_config["init_end_eff_pos"],
                 robot_config["init_end_eff_quat"],
                rgba=[1, 0, 0, 0.4],
                 static=True,
                 visual_only=True,
             )
         )
        
        self.control_step = 1
        self.timer = 0

    def getControl(self, robot: RobotBase):
        if self.timer % self.control_step == 0:
            desired_pos, desired_quat = self.read_ctrl_pos(), self.read_ctrl_quat()
            desired_pos_local = robot._localize_cart_pos(desired_pos)
            desired_quat_local = robot._localize_cart_quat(desired_quat)
            self.setSetPoint(np.hstack((desired_pos_local, desired_quat_local)))
            # comment this line if not using gamepad
            self.scene.set_obj_pos_and_quat(
                desired_pos, desired_quat, obj_name=self.indicator_name
            )

        
        self.timer += 1
        return super().getControl(robot)

    def reset_robot(self):
        pos_local, quat_local = self.robot._localize_cart_coords(
            self.robot_config["init_end_eff_pos"],
            self.robot_config["init_end_eff_quat"],
        )
        self.scene.set_obj_pos_and_quat(
            pos_local, quat_local, obj_name=self.indicator_name
        )

    @abc.abstractmethod
    def read_ctrl_pos(self):
        pass

    @abc.abstractmethod
    def read_ctrl_quat(self):
        pass

