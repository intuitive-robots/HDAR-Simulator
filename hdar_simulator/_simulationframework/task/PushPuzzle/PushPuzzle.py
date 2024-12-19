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


class PushPuzzle(SFSimulator):

    def __init__(self, host_address=None):
        self.box_space = SamplingSpace(
            low=np.array([0.3, -0.3, 0]),
            high=np.array([0.6, 0.3, 0]),
            seed=np.random.randint(0, 1000),
        )
        super().__init__('PushPuzzle', host_address)
        self.haptic_on: bool = False
        self.haptic_on_left: bool = False
        self.haptic_aim_right = False
        self.haptic_aim_left = False

    def create_robots(self) -> Dict[str, MjRobot]:
        
        self.push_robot1 = self.sim_factory.create_robot(
            self.mj_scene,
            xml_path=sim_framework_path("./models/mj/robot/panda.xml"),
            base_position = [0.0, 0.35, 0.0]
        )
        self.push_robot2 = self.sim_factory.create_robot(
            self.mj_scene,
            xml_path=sim_framework_path("./models/mj/robot/panda.xml"),
            base_position = [0.0, -0.35, 0.0]
        )
        return {"panda_robot1": self.push_robot1, 
                "panda_robot2": self.push_robot2}
        

    def create_objects(self) -> Dict[str, MujocoObject]:
        self.picked_box = CustomMujocoObject(
            object_name="pushed_box",
            object_dir_path=os.path.dirname(os.path.abspath(__file__)),
            pos=[0.4, 0.2, 0.0],
            quat=[0, 0, 0, 1],
        )
        self.target_box = CustomMujocoObject(
            object_name="target_box",
            object_dir_path=os.path.dirname(os.path.abspath(__file__)),
            pos=[0.4, -0.2, 0.0],
            quat=[0, 0, 0, 1],
        )
        self.platform_box = CustomMujocoObject(
            object_name="platform_box",
            object_dir_path=os.path.dirname(os.path.abspath(__file__)),
            pos=[0.0, 0.0, 0.0],
            quat=[0, 0, 0, 1],
        )
        return {
            "pushed_box": self.picked_box,
            "target_box": self.target_box,
            "platform_box":self.platform_box,
        }

    def reset(self):
        # return
        pushed_box_pos = self.box_space.sample()
        pushed_box_pos = [pushed_box_pos[0],pushed_box_pos[1],0.13]
        pushed_box_euler = np.array([np.random.uniform(-180, 180), 0, 0])
        pushed_box_quat = R.from_euler("xyz", pushed_box_euler).as_quat()
        self.mj_scene.set_obj_pos_and_quat(
            new_pos=pushed_box_pos,
            new_quat=pushed_box_quat,
            obj_name="pushed_box",
        )
        while True:
            target_box_pos = self.box_space.sample()
            target_box_pos = [target_box_pos[0],target_box_pos[1], 0.13]
            if  0.2 < np.linalg.norm(
                np.array(target_box_pos) - np.array(pushed_box_pos)
            ) < 1.0 :
                break
        # target_box_pos = np.array([np.random.uniform(0.2, 0.5), 0, 0])
        target_box_euler = np.array([np.random.uniform(-180, 180), 0, 0])
        target_box_quat = R.from_euler("xyz", target_box_euler).as_quat()
        self.mj_scene.set_obj_pos_and_quat(
            new_pos=target_box_pos,
            new_quat=target_box_quat,
            obj_name="target_box",
        )
        home_pos = [0.53,-0.09,0.32]
        home_quat = [-0.03,0.75,0.67,0.01]
        self.push_robot1.gotoCartPositionAndQuat(
            desiredPos=[home_pos[0],home_pos[1]+0.35,home_pos[2]],
            desiredQuat=home_quat,
            duration=0.5,
        )
        self.push_robot2.gotoCartPositionAndQuat(
            desiredPos=[home_pos[0],home_pos[1]-0.35,home_pos[2]],
            desiredQuat=home_quat,
            duration=0.5,
        )
 
        # if pushed_box_pos[1] > target_box_pos[1]:
        #     self.push_robot1.gotoCartPositionAndQuat(
        #         desiredPos=[pushed_box_pos[0]+0.1, pushed_box_pos[1]+0.1, 0.4],
        #         desiredQuat=[0, 1, 0, 0],
        #         duration=0.5,
        #     )
            
        #     self.push_robot2.gotoCartPositionAndQuat(
        #         desiredPos=[target_box_pos[0]+0.1, target_box_pos[1]-0.1, 0.4],
        #         desiredQuat=[0, 1, 0, 0],
        #         duration=0.5,
        #     )
        #     self.push_robot1.gotoCartPositionAndQuat(
        #         desiredPos=[pushed_box_pos[0]+0.1, pushed_box_pos[1]+0.1, 0.35],
        #         desiredQuat=[0, 1, 0, 0],
        #         duration=0.2,
        #     )
        #     self.push_robot2.gotoCartPositionAndQuat(
        #         desiredPos=[target_box_pos[0]+0.1, target_box_pos[1]-0.1, 0.35],
        #         desiredQuat=[0, 1, 0, 0],
        #         duration=0.2,
        #     )
        # else:
        #     self.push_robot2.gotoCartPositionAndQuat(
        #         desiredPos=[pushed_box_pos[0]+0.1, pushed_box_pos[1]-0.1, 0.4],
        #         desiredQuat=[0, 1, 0, 0],
        #         duration=0.5,
        #     )
            
        #     self.push_robot1.gotoCartPositionAndQuat(
        #         desiredPos=[target_box_pos[0]+0.1, target_box_pos[1]+0.1, 0.4],
        #         desiredQuat=[0, 1, 0, 0],
        #         duration=0.5,
        #     )
        #     self.push_robot2.gotoCartPositionAndQuat(
        #         desiredPos=[pushed_box_pos[0]+0.1, pushed_box_pos[1]-0.1, 0.35],
        #         desiredQuat=[0, 1, 0, 0],
        #         duration=0.2,
        #     )
        #     self.push_robot1.gotoCartPositionAndQuat(
        #         desiredPos=[target_box_pos[0]+0.1, target_box_pos[1]+0.1, 0.35],
        #         desiredQuat=[0, 1, 0, 0],
        #         duration=0.2,
        #     )
        self.push_robot1.activeController = self.controller_dict["panda_robot1"]
        self.push_robot2.activeController = self.controller_dict["panda_robot2"]
        
        self.push_robot1.activeController.setSetPoint(
            np.hstack((self.push_robot1.current_c_pos, home_quat))
        )

        self.push_robot2.activeController.setSetPoint(
            np.hstack((self.push_robot2.current_c_pos, home_quat))
        )


        inital_pos1 = np.array([home_pos[0],home_pos[1]+0.35,home_pos[2]])
        inital_pos2 = np.array([home_pos[0],home_pos[1]-0.35,home_pos[2]])

        target_pos= [inital_pos1,inital_pos2]
        return target_pos
    

    def before_step(self):
        pass

    def after_step(self):
        pass


class PushPuzzleVibration(PushPuzzle):
    def __init__(self, host_address=None):
        super().__init__(host_address)

    def after_step(self):
        
        replace_rb0_l=0
        replace_rb0_r=0
        replace_rb0_l, replace_rb0_r  = get_collisions(
        self.mj_scene,
        target_pairs1={
            ('pushed_box', 'finger1_rb0_tip_collision'),
            ('pushed_box_part', 'finger1_rb0_tip_collision'),
            ('pushed_box2', 'finger1_rb0_tip_collision'),
            ('pushed_box2_part', 'finger1_rb0_tip_collision'),
            ('pushed_box', 'panda_rb0_leftfinger:geom2'),
            ('pushed_box_part', 'panda_rb0_leftfinger:geom2'),
            ('pushed_box2', 'panda_rb0_leftfinger:geom2'),
            ('pushed_box2_part', 'panda_rb0_leftfinger:geom2'),
            
                       },
        target_pairs2={
            ('pushed_box', 'finger2_rb0_tip_collision'),
            ('pushed_box_part', 'finger2_rb0_tip_collision'),
            ('pushed_box2', 'finger2_rb0_tip_collision'),
            ('pushed_box2_part', 'finger2_rb0_tip_collision'),
            ('pushed_box', 'panda_rb0_rightfinger:geom2'),
            ('pushed_box_part', 'panda_rb0_rightfinger:geom2'),
            ('pushed_box2', 'panda_rb0_rightfinger:geom2'),
            ('pushed_box2_part', 'panda_rb0_rightfinger:geom2'),
          
            }
        )
        replace_rb1_l=0
        replace_rb1_r=0
        replace_rb1_l, replace_rb1_r  = get_collisions(
        self.mj_scene,
        target_pairs1={
            ('pushed_box', 'finger1_rb1_tip_collision'),
            ('pushed_box_part', 'finger1_rb1_tip_collision'),
            ('pushed_box2', 'finger1_rb1_tip_collision'),
            ('pushed_box2_part', 'finger1_rb1_tip_collision'),
            ('pushed_box', 'panda_rb1_leftfinger:geom2'),
            ('pushed_box_part', 'panda_rb1_leftfinger:geom2'),
            ('pushed_box2', 'panda_rb1_leftfinger:geom2'),
            ('pushed_box2_part', 'panda_rb1_leftfinger:geom2'),
            
                       },
        target_pairs2={
            
            ('pushed_box', 'finger2_rb1_tip_collision'),
            ('pushed_box_part', 'finger2_rb1_tip_collision'),
            ('pushed_box2', 'finger2_rb1_tip_collision'),
            ('pushed_box2_part', 'finger2_rb1_tip_collision'),
            ('pushed_box', 'panda_rb1_rightfinger:geom2'),
            ('pushed_box_part', 'panda_rb1_rightfinger:geom2'),
            ('pushed_box2', 'panda_rb1_rightfinger:geom2'),
            ('pushed_box2_part', 'panda_rb1_rightfinger:geom2'),
          
            }
        )

        if replace_rb0_l != 0 or replace_rb0_r !=0:
            self.haptic_on_right = True
            
        else:
            self.haptic_on_right = False
            
        if replace_rb1_l != 0 or replace_rb1_r !=0:
            self.haptic_on_left = True
            
        else:
            self.haptic_on_left = False

