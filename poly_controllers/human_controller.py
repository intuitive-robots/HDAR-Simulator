import torch
import torchcontrol as toco

from typing import Dict
from polymetis import RobotInterface


class HumanController(toco.PolicyModule):
    # stolen from SimulationFramework
    def __init__(self, robot: RobotInterface, regularize=True):
        super().__init__()

        # get joint limits for regularization
        limits = robot.robot_model.get_joint_angle_limits()
        self.joint_pos_min = limits[0]
        self.joint_pos_max = limits[1]

        # define gain
        self.gain = torch.Tensor([0.26, 0.44, 0.40, 1.11, 1.10, 1.20, 0.85])

        if regularize:
            self.reg_gain = torch.Tensor([5.0, 2.2, 1.3, 0.3, 0.1, 0.1, 0.0])
        else:
            self.reg_gain = torch.Tensor([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def forward(self, state_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        ext = state_dict["motor_torques_external"]

        human_torque = -self.gain * ext

        joint_pos_current = state_dict["joint_positions"]

        left_boundary = 1 / torch.clamp(
            torch.abs(self.joint_pos_min - joint_pos_current), 1e-8, 100000
        )
        right_boundary = 1 / torch.clamp(
            torch.abs(self.joint_pos_max - joint_pos_current), 1e-8, 100000
        )

        reg_load = left_boundary - right_boundary

        reg_torgue = self.reg_gain * reg_load

        return {"joint_torques": human_torque + reg_torgue}