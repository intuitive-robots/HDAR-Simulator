from typing import Dict
from scipy.spatial.transform import Rotation as R
import numpy as np
import os
from gym.spaces import Box as SamplingSpace

from ...sf_simulator import SFSimulator
from alr_sim.utils.sim_path import sim_framework_path

from alr_sim.sims.mj_beta import MjRobot
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import MujocoObject
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CustomMujocoObject
from ..Collision_finger import get_collisions , aim_resultant_force

class OpenDrawer(SFSimulator):

    def __init__(self,host_address=None):
        self.box_space = SamplingSpace(
            low=np.array([0.3, -0.3, 0]),
            high=np.array([0.6, 0.3, 0]),
            seed=np.random.randint(0, 1000),
        )
        super().__init__('OpenDrawer', host_address)
        self.haptic_on: bool = False
        self.haptic_aim: bool = False

    def create_robots(self) -> Dict[str, MjRobot]:
        self.pick_robot = self.sim_factory.create_robot(
            self.mj_scene,
            xml_path=sim_framework_path("./models/mj/robot/panda.xml"),
        )
        return {"panda_robot": self.pick_robot}

    def create_objects(self) -> Dict[str, MujocoObject]:
        self.picked_box = CustomMujocoObject(
            object_name="picked_box",
            object_dir_path=os.path.dirname(os.path.abspath(__file__)),
            pos=[0.4, 0.3, 0.01],
            quat=[0, 0, 0, 1],
        )
        self.target_box = CustomMujocoObject(
            object_name="target_box",
            object_dir_path=os.path.dirname(os.path.abspath(__file__)),
            pos=[0.4, 0.3, 0.05],
            quat=[0, 0, 0, 1],
        )
        return {
            "picked_box": self.picked_box,
            "target_box": self.target_box,
        }


    def reset(self):
        while True:
            picked_box_pos = [np.random.uniform(0.35, 0.5),np.random.uniform(-0.6, 0.6),0]
            if 0.6 < np.linalg.norm(
               np.array(picked_box_pos) - np.array([0,0,0.06])
            ) < 0.8 :
                break
        if picked_box_pos[1] < 0:
            picked_box_euler = np.array([0.15, 0, 0])
        else: 
            picked_box_euler = np.array([-3, 0, 0])
        # angle1 = np.arctan2(picked_box_pos[1], picked_box_pos[0])

        # picked_box_euler = np.array([-1.57-angle1, 0, 0])
        picked_box_quat = R.from_euler("xyz", picked_box_euler).as_quat()
        self.mj_scene.set_obj_pos_and_quat(
            new_pos=picked_box_pos,
            new_quat=picked_box_quat,
            obj_name="picked_box",
        )
        
        target_box_quat = R.from_euler("xyz", picked_box_euler).as_quat()
        target_box_pos= picked_box_pos+np.array([0,0,0.06])
        self.mj_scene.set_obj_pos_and_quat(
            new_pos=target_box_pos,
            new_quat=target_box_quat,
            obj_name="target_box",
        )
        current_pos = self.pick_robot.current_c_pos
        home_pos = [0.53,-0.09,0.32]
        home_quat = [-0.03,0.75,0.67,0.01]
        self.pick_robot.gotoCartPositionAndQuat(
            desiredPos=[current_pos[0], current_pos[1], home_pos[2]],
            desiredQuat=home_quat,
            duration=0.2,
        )
        self.pick_robot.gotoCartPositionAndQuat(
            desiredPos=home_pos,
            desiredQuat=home_quat,
            duration=0.5,
        )
        inital_pos = home_pos
        self.pick_robot.activeController = self.controller_dict["panda_robot"]
        self.pick_robot.activeController.setSetPoint(
            np.hstack((home_pos, home_quat))
        )
        real_reset_target=[inital_pos]

        return real_reset_target
    def before_step(self):
        pass

    def after_step(self):
        pass


class OpenDrawerVibration(OpenDrawer):

    def __init__(self, host_address=None):
        super().__init__(host_address)
        

    def after_step(self):
               
        replace_rb0_l=0
        replace_rb0_r=0
        replace_rb0_l, replace_rb0_r  = get_collisions(
        self.mj_scene,
        target_pairs1={
            ('picked_handle1', 'finger1_rb0_tip_collision'),
            ('picked_handle2', 'finger1_rb0_tip_collision'),
            ('picked_handle', 'finger1_rb0_tip_collision'),
        },
        target_pairs2={
            ('picked_handle1', 'finger2_rb0_tip_collision'),
            ('picked_handle2', 'finger1_rb0_tip_collision'),
            ('picked_handle', 'finger2_rb0_tip_collision'),
        }
        )
        if (replace_rb0_l != 0 or replace_rb0_r !=0) :
            self.haptic_on = True
        else:
            self.haptic_on = False


                  