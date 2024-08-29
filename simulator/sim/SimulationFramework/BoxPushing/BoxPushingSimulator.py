from typing import Dict
from alr_sim.controllers import ControllerBase
from scipy.spatial.transform import Rotation as R
import numpy as np
import os

from ..SFSimulator import SFSimulator
from alr_sim.utils.sim_path import sim_framework_path
from simpub.xr_device.meta_quest3 import MetaQuest3
from alr_sim.controllers.IKControllers import CartPosQuatImpedenceController
from alr_sim.sims.mj_beta import MjRobot
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import MujocoObject
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CustomMujocoObject


class MetaQuest3Controller(CartPosQuatImpedenceController):

    def __init__(self, device, fix_rotation=False, with_hand=True):
        super().__init__()
        self.device: MetaQuest3 = device
        self.fix_rotation = fix_rotation
        self.with_hand = with_hand

    def getControl(self, robot: MjRobot):
        input_data = self.device.get_input_data()
        if input_data is None:
            return super().getControl(robot)
        hand = input_data["right"]
        # pos and quat offsets
        desired_pos = hand["pos"]
        print(desired_pos)
        desired_pos[0] = desired_pos[0] - 0.2
        desired_pos_local = robot._localize_cart_pos(desired_pos)
        if self.fix_rotation:
            desired_quat_local = np.array([0, 1, 0, 0])
        else:
            desired_quat = hand["rot"]
            rot = R.from_quat(desired_quat) * R.from_euler(
                "xyz", [-180, 0, 180], True
            )
            desired_quat = rot.as_quat(scalar_first=True)
            desired_pos_local = robot._localize_cart_quat(desired_quat)
        if self.with_hand:
            if hand["index_trigger"]:
                robot.close_fingers(duration=0.0)
            else:
                robot.open_fingers()
        self.setSetPoint(np.hstack((desired_pos_local, desired_quat_local)))
        return super().getControl(robot)


class BoxPushingSimulator(SFSimulator):

    def __init__(self):
        super().__init__('BoxPushing')

    def create_robots(self) -> Dict[str, MjRobot]:
        push_robot = self.sim_factory.create_robot(
            self.mj_scene,
            xml_path=sim_framework_path("./models/mj/robot/panda_rod.xml"),
        )
        return {"push_robot": push_robot}

    def create_objects(self) -> Dict[str, MujocoObject]:
        pushed_box = CustomMujocoObject(
            object_name="pushed_box",
            object_dir_path=".",
            pos=[0.4, 0, 0.3],
            quat=[0, 0, 0, 1],
            root=os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "../models/",
            )
        )
        return {"pushed_box": pushed_box}

    def create_controller(self) -> Dict[str, ControllerBase]:
        self.device = MetaQuest3("ALRMetaQuest3")
        controller = MetaQuest3Controller(
            self.device, fix_rotation=True, with_hand=False
        )
        return {"push_robot": controller}

    def after_step(self):
        input_data = self.device.get_input_data()
        if input_data is not None and input_data["X"] is True:
            self.recorder.save_record()


if __name__ == '__main__':
    simulator = BoxPushingSimulator()
    simulator.run()
