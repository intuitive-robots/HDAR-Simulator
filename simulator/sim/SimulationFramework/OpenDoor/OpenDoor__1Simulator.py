from typing import Dict
from alr_sim.controllers import ControllerBase
from scipy.spatial.transform import Rotation as R
import numpy as np
import os
from gym.spaces import Box as SamplingSpace

from ..SFSimulator import SFSimulator
from alr_sim.utils.sim_path import sim_framework_path
from simpub.xr_device.meta_quest3 import MetaQuest3
from alr_sim.controllers.IKControllers import CartPosQuatImpedenceController
from alr_sim.sims.mj_beta import MjRobot
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import MujocoObject
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CustomMujocoObject
from ..Collision_finger import Collision_finger , Collision_aim
import time

class MetaQuest3Controller(CartPosQuatImpedenceController):

    def __init__(
        self,
        device,
        fix_rotation=False,
        with_hand=True,
        hand_side="right"
    ):
        super().__init__()
        self.device: MetaQuest3 = device
        self.fix_rotation = fix_rotation
        self.with_hand = with_hand
        self.hand_side = hand_side
        self.on_control = False
        self.start_pos_offset = None

    def getControl(self, robot: MjRobot):
        input_data = self.device.get_input_data()
        if input_data is None:
            return super().getControl(robot)
        hand = input_data[self.hand_side]
        if hand["hand_trigger"] is False:
            self.on_control = False
            return super().getControl(robot)
        # initial position
        if self.on_control is False:
            self.start_pos_offset = (
                robot.current_c_pos_global - np.array(hand["pos"])
            )
            self.on_control = True
        # pos and quat offsets
        desired_pos = np.array(hand["pos"]) + self.start_pos_offset
        desired_pos_local = robot._localize_cart_pos(desired_pos)
        if self.fix_rotation==True:
            desired_quat_local = np.array([0, 1, 0, 0])
            desired_pos[2] = 0.13
        else:
            desired_quat = hand["rot"]
        rot = R.from_quat(desired_quat) * R.from_euler(
            "xyz", [-180, 0, 180], True
        )
        desired_quat = rot.as_quat(scalar_first=True)
        desired_quat_local = robot._localize_cart_quat(desired_quat)

        if self.with_hand:
            if hand["index_trigger"]:
                robot.close_fingers(duration=0.0)
            else:
                robot.open_fingers()
        self.setSetPoint(np.hstack((desired_pos_local, desired_quat_local)))
        return super().getControl(robot)


class OpenDoorSimulator(SFSimulator):

    def __init__(self):
        self.box_space = SamplingSpace(
            low=np.array([0.3, -0.3, 0]),
            high=np.array([0.6, 0.3, 0]),
            seed=np.random.randint(0, 1000),
        )
        super().__init__(
            'OpenDoor',
            host_address="192.168.0.134",
        )
        self.vibration_triggered = False
        self.on_vibration = False

    def create_robots(self) -> Dict[str, MjRobot]:
        
        self.pick_robot1 = self.sim_factory.create_robot(
            self.mj_scene,
            xml_path=sim_framework_path("./models/mj/robot/panda.xml"),
              )
        
        return {"pick_robot1": self.pick_robot1}
        

    def create_objects(self) -> Dict[str, MujocoObject]:
        self.picked_Box = CustomMujocoObject(
            object_name="picked_box",
            object_dir_path=os.path.dirname(os.path.abspath(__file__)),
            pos=[0.4, 0, 0.3],
            quat=[0, 0, 0, 1],
        )
        self.target_Box = CustomMujocoObject(
            object_name="target_box",
            object_dir_path=os.path.dirname(os.path.abspath(__file__)),
            pos=[0.4, 0.3, 0.3],
            quat=[0, 0, 0, 1],
        )
        
        return {
            "picked_box": self.picked_Box,
            "target_box": self.target_Box,
            
        }

    def create_controller(self) -> Dict[str, ControllerBase]:
        self.device = MetaQuest3("ALRMetaQuest2")
        self.device.register_button_trigger_event("X", self.recorder.save_record)
        self.device.register_button_trigger_event("X", self.put_reset_main_thread)
        self.controller1 = MetaQuest3Controller(self.device, fix_rotation=False, with_hand=True, hand_side="right")
        

        return {
            "pick_robot1": self.controller1,
        }
            

    def after_step(self):
        input_data = self.device.get_input_data()
        if input_data is None:
            return
        elif input_data["right"]["hand_trigger"] is True:
            self.recorder.start_record()
        else:
            self.recorder.stop_record()
        
        replace_rb0_l=0
        replace_rb0_r=0
        self.rb0_finger_collision = Collision_finger(
        self.mj_scene,
        target_pairs1={('picked_handle', 'finger1_rb0_tip_collision')},
        target_pairs2={('picked_handle', 'finger2_rb0_tip_collision')}
        )
        replace_rb0_l, replace_rb0_r = self.rb0_finger_collision.get_collisions()
        # self.collision_aim = Collision_aim(
        # self.mj_scene,
        # target_pairs={},
        # )
        # replace_aim=self.collision_aim.aim_resultant_force()
        hand = input_data["right"]
        if replace_rb0_l != 0 or replace_rb0_r !=0:
            if hand["index_trigger"] :
                if not self.vibration_triggered:
                    self.device.start_vibrate()
                    time.sleep(0.01)
                    print("vibrate because contact with gripping")
                    self.vibration_triggered = True
                    self.on_vibration = True
                    # self.device.stop_vibrate_now()
                else: 
                    print("not vibrate because contact with gripping")
                    if self.on_vibration:
                        self.device.stop_vibrate_now()
                        self.on_vibration = False
                    # if replace_aim != 0 :
                    #     print("collision with enviornment(aim object)")
                    #     self.device.start_vibrate()
                   

            elif not hand["index_trigger"] and self.on_vibration == False:
                self.device.start_vibrate()
                # self.device.stop_vibrate_now()
                print("vibrate because contact without gripping")
                self.vibration_triggered = False
                self.on_vibration = True
                
        else:
            if self.on_vibration:
                self.device.stop_vibrate_now()
                time.sleep(0.01)
                self.on_vibration = False

            elif not hand["index_trigger"]:
                self.vibration_triggered = False
            
            


                
                

       

    def put_reset_main_thread(self):
        self.callback_task_list.append(self.reset)

    def reset(self):
        
        pushed_box_pos = self.box_space.sample()
        pushed_box_euler = np.array([np.random.uniform(-180, 180), 0, 0])
        pushed_box_quat = R.from_euler("xyz", pushed_box_euler).as_quat()
        self.mj_scene.set_obj_pos_and_quat(
            new_pos=pushed_box_pos,
            new_quat=pushed_box_quat,
            obj_name="picked_box",
        )
        
        target_box_quat = R.from_euler("xyz", pushed_box_euler).as_quat()
        target_box_pos= pushed_box_pos+np.array([0,0,0.06])
        self.mj_scene.set_obj_pos_and_quat(
            new_pos=target_box_pos,
            new_quat=target_box_quat,
            obj_name="target_box",
        )
        self.pick_robot1.gotoCartPositionAndQuat(
            desiredPos=[self.picked_Box.pos[0], self.picked_Box.pos[1], 0.3],
            desiredQuat=[0, 1, 0, 0],
            duration=2.0,
        )
        self.pick_robot1.gotoCartPositionAndQuat(
            desiredPos=[self.picked_Box.pos[0], self.picked_Box.pos[1], 0.13],
            desiredQuat=[0, 1, 0, 0],
            duration=2.0,
        )
      
        self.pick_robot1.activeController = self.controller1
        
        self.controller1.setSetPoint(
            np.hstack((self.pick_robot1.current_c_pos_global, [0, 1, 0, 0]))
        )
        


if __name__ == '__main__':
    simulator = OpenDoorSimulator()
    simulator.run()
