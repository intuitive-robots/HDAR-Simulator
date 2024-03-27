from alr_sim import controllers
import numpy as np
from alr_sim.core import RobotBase


class ReplayerController(controllers.JointPDController):
    def __init__(
        self,
        replay_data,
        robot: RobotBase,
        downsample_steps: float = 50,
        initial_index=0,
    ):
        super().__init__()
        self.replay_data = replay_data
        self.timer = 0
        self.index = initial_index
        self.downsample_steps = downsample_steps
        self.robot = robot
        self.desired_joint_pos_array = np.array(replay_data["des_joint_pos"])
        self.desired_joint_vel_array = np.array(replay_data["des_joint_vel"])
        self.desired_joint_acc_array = np.array(replay_data["des_joint_acc"])
        self.gripper_width_array = np.array(replay_data["gripper_width"])
        self.sequence_length = len(self.desired_joint_pos_array)

    def getControl(self, robot: RobotBase):
        if self.timer % self.downsample_steps == 0:
            self.desired_joint_pos = self.desired_joint_pos_array[self.index]
            self.desired_joint_vel = self.desired_joint_vel_array[self.index]
            self.desired_joint_acc = self.desired_joint_acc_array[self.index]
            if self.gripper_width_array[self.index] <= 0.075:
                robot.close_fingers(duration=0)
            else:
                robot.open_fingers()
            self.index += 1
        self.timer += 1
        return super().getControl(robot)

    def reset_robot(self):
        self.robot.beam_to_joint_pos(self.desired_joint_pos_array[0])

    def isSequenceFinished(self):
        return self.index >= self.sequence_length