from typing import List, Dict, TypedDict
import numpy as np
import os

from alr_sim.controllers.Controller import JointPDController
from alr_sim.sims.mj_beta import MjRobot
from .SFSimulator import SFSimulator
from ..data_replayer import DataReplayer
from .task import sf_task_factory




class DemonstrationHeader(TypedDict):
    task: str
    record_step: int
    simulation_dt: float


class Demonstration(TypedDict):
    header: str
    init_objects_state: Dict[str, Dict[str, List[float]]]
    init_robots_state: Dict[str, Dict[str, List[float]]]
    data: List[Dict[str, List[float]]]
    record_step: int


class SFReplayerJointController(JointPDController):
    def __init__(
        self,
        demonstration: Demonstration,
        robot: MjRobot,
        robot_name: str,
        downsample_steps: float = 50,
    ):
        super().__init__()
        self.demonstration = demonstration
        self.sequence_length = len(demonstration["data"])
        robot_data = np.zeros((self.sequence_length, 7))
        self.timer = 0
        self.index = 0
        for index, frame in enumerate(demonstration["data"]):
            robot_data[index, :] = np.array(frame[robot_name]["joint_pos"])
        self.robot_data = robot_data
        self.downsample_steps = downsample_steps
        self.robot = robot

    def getControl(self, robot: MjRobot):
        if self.timer % self.downsample_steps == 0:
            self.desired_joint_pos = self.robot_data[self.index]
            # self.desired_joint_vel = self.desired_joint_vel_array[self.index]
            # self.desired_joint_acc = self.desired_joint_acc_array[self.index]
            # if self.gripper_width_array[self.index] <= 0.075:
            #     robot.close_fingers(duration=0)
            # else:
            #     robot.open_fingers()
            self.index += 1
            print(f"Index: {self.index}")
        self.timer += 1
        return super().getControl(robot)

    # def reset_robot(self):
    #     self.robot.beam_to_joint_pos(self.desired_joint_pos_array[0])

    def isSequenceFinished(self):
        return self.index >= self.sequence_length


class SFReplayer(DataReplayer):

    def __init__(self) -> None:
        super().__init__()

    def create_simulator(self, demo: Demonstration) -> SFSimulator:
        task_name = demo["header"]["task"]
        self.sequence_length = len(demo["data"])
        print(f"Creating simulation for task: {task_name}")
        self.simulator: SFSimulator = sf_task_factory(
            task_name, record_mode=False
        )
        self.mj_scene = self.simulator.mj_scene
        self.simulator.reset_objects(demo["init_objects_state"])
        self.simulator.reset_robots(demo["init_robots_state"])
        controller_dict = {}
        for robot_name, robot in self.simulator.robot_dict.items():
            controller_dict[robot_name] = SFReplayerJointController(
                demo, robot, robot_name,
            )
        self.simulator.assign_controller(controller_dict)
        return self.simulator

    def start_replay(self):
        # self.simulator.mj_scene.start()
        pass

    def replay_step(self):
        self.mj_scene.next_step()
        super().replay_step()