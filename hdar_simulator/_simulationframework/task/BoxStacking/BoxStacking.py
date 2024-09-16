from typing import Dict
from scipy.spatial.transform import Rotation as R
import numpy as np
import os
from gym.spaces import Box as SamplingSpace

from ...sf_simulator import SFSimulator
from alr_sim.utils.sim_path import sim_framework_path

from alr_sim.core.Scene import Scene
from alr_sim.sims.mj_beta import MjRobot
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import MujocoObject
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CustomMujocoObject
from alr_sim.sims.universal_sim.PrimitiveObjects import Box

class BoxStacking(SFSimulator):

    def __init__(self, host_address=None, render=Scene.RenderMode.HUMAN):
        self.box_space = SamplingSpace(
            low=np.array([0.3, -0.3, 0]),
            high=np.array([0.6, 0.3, 0]),
            seed=np.random.randint(0, 1000),
        )
        super().__init__('BoxStacking', host_address, render)

    def create_robots(self) -> Dict[str, MjRobot]:
        self.push_robot = self.sim_factory.create_robot(
            self.mj_scene,
            xml_path=sim_framework_path("./models/mj/robot/panda.xml"),
        )
        return {"panda_gripper_robot": self.push_robot}

    def create_objects(self) -> Dict[str, MujocoObject]:

        self.red_box = Box(
            name="red_box",
            init_pos=np.array([0.5, -0.1, 0.0]),
            init_quat=[0, 1, 0, 0],
            rgba=[1, 0, 0, 1.0],
            mass=0.05,
            size=[0.03, 0.03, 0.03],
            # visual_only=True,
        )

        self.green_box = Box(
            name="green_box",
            init_pos=np.array([0.5, 0, 0.0]),
            init_quat=[0, 1, 0, 0],
            rgba=[0, 1, 0, 1.0],
            mass=0.05,
            size=[0.03, 0.03, 0.03],
            # visual_only=True,
        )

        self.blue_box = Box(
            name="blue_box",
            init_pos=np.array([0.5, 0.1, 0.0]),
            init_quat=[0, 1, 0, 0],
            rgba=[0, 0, 1, 1.0],
            mass=0.05,
            size=[0.03, 0.05, 0.03],
            # visual_only=True,
        )

        self.target_box = Box(
            name="target_box",
            init_pos=[0.5, 0.2, 0],
            init_quat=[0, 1, 0, 0],
            size=[0.05, 0.05, 0.04],
            rgba=[1, 0.65, 0, 0.3],
            visual_only=True,
            static=True
            # wall_height=0.005,
        )

        return {
            "red_box": self.red_box,
            "green_box": self.green_box,
            "blue_box": self.blue_box,
            "target_box": self.target_box,
        }

    def reset(self):
        return
        pushed_box_pos = self.box_space.sample()
        pushed_box_euler = np.array([np.random.uniform(-180, 180), 0, 0])
        pushed_box_quat = R.from_euler("xyz", pushed_box_euler).as_quat()
        self.mj_scene.set_obj_pos_and_quat(
            new_pos=pushed_box_pos,
            new_quat=pushed_box_quat,
            obj_name="pushed_box",
        )
        while True:
            target_box_pos = self.box_space.sample()
            if np.linalg.norm(
                np.array(target_box_pos) - np.array(pushed_box_pos)
            ) > 0.2:
                break
        target_box_euler = np.array([np.random.uniform(-180, 180), 0, 0])
        target_box_quat = R.from_euler("xyz", target_box_euler).as_quat()
        self.mj_scene.set_obj_pos_and_quat(
            new_pos=target_box_pos,
            new_quat=target_box_quat,
            obj_name="target_box",
        )
        current_pos = self.push_robot.current_c_pos
        self.push_robot.gotoCartPositionAndQuat(
            desiredPos=[current_pos[0], current_pos[1], 0.25],
            desiredQuat=[0, 1, 0, 0],
            duration=2.0,
        )
        self.push_robot.gotoCartPositionAndQuat(
            desiredPos=[pushed_box_pos[0], pushed_box_pos[1], 0.25],
            desiredQuat=[0, 1, 0, 0],
            duration=2.0,
        )
        self.push_robot.gotoCartPositionAndQuat(
            desiredPos=[pushed_box_pos[0], pushed_box_pos[1], 0.13],
            desiredQuat=[0, 1, 0, 0],
            duration=2.0,
        )
        inital_pos = np.array([pushed_box_pos[0], pushed_box_pos[1], 0.13])
        self.push_robot.activeController = self.controller_dict["panda_gripper_robot"]
        self.push_robot.activeController.setSetPoint(
            np.hstack((inital_pos, np.array([0, 1, 0, 0])))
        )
        # reset again
        # self.mj_scene.set_obj_pos_and_quat(
        #     new_pos=pushed_box_pos,
        #     new_quat=pushed_box_quat,
        #     obj_name="pushed_box",
        # )

    def before_step(self):
        pass

    def after_step(self):
        pass

    # def is_done(self, error: int = 0.05):
    #     push_box_pos = self.mj_scene.get_obj_pos(obj_name="pushed_box")
    #     target_box_pos = self.mj_scene.get_obj_pos(obj_name="target_box")
    #     distance = np.linalg.norm(np.array(push_box_pos) - np.array(target_box_pos))
    #     return distance < error