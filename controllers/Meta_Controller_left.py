import numpy as np
from alr_sim.controllers.IKControllers import CartPosQuatImpedenceController
from simpub.xr_device.meta_quest3 import MetaQuest3
from scipy.spatial.transform import Rotation as R
from alr_sim.sims.mj_beta import MjRobot

class MetaQuest3Controller_left(CartPosQuatImpedenceController):

    def __init__(self, device):
        super().__init__()
        self.device: MetaQuest3 = device

    def getControl(self, robot):
        input_data = self.device.get_input_data()
        if input_data is not None:
            hand = input_data["left"]
            desired_pos = hand["pos"]
            pos_offset = np.array([-0.2,0.0,0.0]) 
            desired_quat = hand["rot"]
            if desired_quat != [0,0,0,0]:
                rot = R.from_quat(desired_quat) * R.from_euler("xyz", [-180, 0, 180], True)
                desired_quat = rot.as_quat(scalar_first=True)
            desired_pos_local = robot._localize_cart_pos(desired_pos)+pos_offset
            desired_quat_local = robot._localize_cart_quat(desired_quat)
            if hand["index_trigger"]:
                robot.close_fingers(duration=0.0)
            else:
                robot.open_fingers()
            self.setSetPoint(np.hstack((desired_pos_local, desired_quat_local)))
        return super().getControl(robot)
