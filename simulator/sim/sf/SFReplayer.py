from typing import List, Dict, TypedDict
import numpy as np
import os

from alr_sim.controllers.Controller import JointPDController
from alr_sim.sims.mj_beta import MjRobot
from .SFSimulator import SFSimulator
from ..data_replayer import DataReplayer
from .task import sf_task_factory


class SFReplayerJointController(JointPDController):
    def __init__(
        self,
        replay_data: dict[dict[list]],
        robot: MjRobot,
        downsample_steps: float = 50,
        start_index=0,
    ):
        super().__init__()
        self.replay_data = replay_data
        self.timer = 0
        self.index = start_index
        self.downsample_steps = downsample_steps
        self.robot = robot
        self.desired_joint_pos_array = np.array(replay_data["des_joint_pos"])
        self.desired_joint_vel_array = np.array(replay_data["des_joint_vel"])
        self.desired_joint_acc_array = np.array(replay_data["des_joint_acc"])
        self.gripper_width_array = np.array(replay_data["gripper_width"])
        self.sequence_length = len(self.desired_joint_pos_array)

    def getControl(self, robot: MjRobot):
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


class DemonstrationHeader(TypedDict):
    task: str
    record_step: int
    simulation_dt: float


class Demonstration(TypedDict):
    header: str
    init_objects_state: Dict[str, Dict[str, List[float]]]
    init_robots_state: Dict[str, Dict[str, List[float]]]
    data: List[Dict]
    record_step: int


class SFReplayer(DataReplayer):

    def __init__(self) -> None:
        super().__init__()

    def create_simulation(self, demo: Dict):
        task_name = demo["header"]["task"]
        print(f"Creating simulation for task: {task_name}")
        self.simulator: SFSimulator = sf_task_factory(task_name, record_mode=False)
        self.mj_scene = self.simulator.mj_scene
        self.simulator.reset_objects(demo["init_objects_state"])
        self.simulator.reset_robots(demo["init_robots_state"])

    def start_replay(self):
        # self.simulator.mj_scene.start()
        pass

    def replay_step(self):
        self.mj_scene.next_step()
