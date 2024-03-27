import torch
import torchcontrol as toco

from typing import Dict
from polymetis import RobotInterface


def total_force_clip(
    replica_load, max_force: float = 8.0, a: float = 6.0, b: float = 4.0
):
    """
    computes l1 norm of the replica load (maybe with incorporated d-gain control of the primary velocity).
    If it is smaller than b: return zero force
    If it is between a and b: return linear interpolation
    If it is bigger than a: return replica_load
    if it is bigger than max_force: scale it down such that total force == max_force
    """
    total_force = torch.sum(torch.abs(replica_load))
    if total_force < b:
        return torch.zeros(7)
    elif total_force < a:
        return (1 / (a - b) * total_force - b / (a - b)) * replica_load
    elif total_force < max_force:
        return replica_load
    else:
        return replica_load * (max_force / total_force)


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


class ForceFeedbackController(toco.PolicyModule):
    # stolen from SimulationFramework
    def __init__(self, robot: RobotInterface, regularize=True):
        super().__init__()
        self.human_control = HumanController(robot, regularize)

        self.fb_gain = -1.0 * torch.tensor([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        self.torque_dgain = 0.5 * torch.tensor([50.0, 50.0, 50.0, 50.0, 15.0, 8.0, 8.0])
        self.tau_moving_average = torch.zeros(7)
        self.avg_load = torch.zeros(7)
        self.alpha = 0.9
        self.smoothing_force = 0.95

        self.virtual_load = torch.nn.Parameter(torch.zeros((7,)))

    def forward(self, state_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        ext = state_dict["motor_torques_external"]

        self.avg_load = (
            self.avg_load * self.smoothing_force
            + (1 - self.smoothing_force) * self.virtual_load
        )

        plain_feedback = total_force_clip(
            self.fb_gain * self.avg_load, a=4.66, b=4.0, max_force=60.0
        )

        primary_j_vel = state_dict["joint_velocities"]
        d_feedback = self.torque_dgain * primary_j_vel
        # update moving average of the latest external forces
        self.tau_moving_average = self.alpha * self.tau_moving_average + (
            1 - self.alpha
        ) * torch.abs(plain_feedback)
        # 4. compute clipping boundaries for d_feedback
        m = torch.maximum(torch.abs(plain_feedback), self.tau_moving_average)
        # clip and return
        d_feedback = torch.clip(d_feedback, -m, m)
        human_torque = self.human_control.forward(state_dict)["joint_torques"]
        return {"joint_torques": human_torque + plain_feedback - d_feedback}


class ImitationController(toco.policies.HybridJointImpedanceControl):
    def __init__(self, robot: RobotInterface):
        super().__init__(
            joint_pos_current=robot.get_joint_positions(),
            Kq=robot.Kq_default,
            Kqd=robot.Kqd_default,
            Kx=robot.Kx_default,
            Kxd=robot.Kxd_default,
            robot_model=robot.robot_model,
            ignore_gravity=robot.use_grav_comp,
        )
