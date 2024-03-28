import torch
import torchcontrol as toco

from typing import Dict
from polymetis import RobotInterface
from .human_controller import HumanController


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


class ForceFeedbackController(toco.PolicyModule):
    # stolen from SimulationFramework
    def __init__(self, robot: RobotInterface, regularize=True):
        super().__init__()
        self.human_control = HumanController(robot, regularize)

        self.fb_gain = -1.0 * torch.tensor([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        self.torque_dgain = 0.5 * torch.tensor([50.0, 50.0, 50.0, 50.0, 15.0, 8.0, 8.0])
        self.tau_moving_average = torch.zeros(7)
        self.avg_feedback = torch.zeros(7)
        self.alpha = 0.9
        self.smoothing_force = 0.95

        self.constraint_forces = torch.nn.Parameter(torch.zeros((7,)))

    def forward(self, state_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        ext = state_dict["motor_torques_external"]

        feedback = -self.constraint_forces / 5 

        self.avg_feedback = (
            self.avg_feedback * self.smoothing_force
            + (1 - self.smoothing_force) * feedback
        )

        plain_feedback = total_force_clip(
            self.fb_gain * self.avg_feedback, a=4.66, b=4.0, max_force=60.0
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
