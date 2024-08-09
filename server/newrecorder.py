import os
import pickle
from datetime import datetime

from alr_sim.core import Scene, RobotBase
from alr_sim.core.sim_object import SimObject
from alr_sim.core.logger import ObjectLogger
from alr_sim.utils.sim_path import sim_framework_path

import multiprocessing as mp
from typing import List


class UnityRecorder:
    def __init__(
        self,
        scene: Scene,
        obj_list: List[SimObject],
        task_type,
        manager,
        save_root_path="./ARHumanDemoData/",
        record_mode=True,
        downsample_steps=10,
    ) -> None:
        self.save_root_path = save_root_path
        self.record_mode = record_mode
        self.scene = scene
        self.robots: RobotBase = scene.robots
        self.obj_list = obj_list
        self.task_type = task_type
        self.downsample_steps = downsample_steps

        self.obj_log_dict = dict()
        for obj in self.obj_list:
            self.obj_log_dict[obj.name] = {
                "pos": [],
                "orientation": [],
                "time_stamp": []
            }
        
        self.robot_log_dict = dict()
        for robot in self.robots:
            self.robot_log_dict[robot] = {
                "joint_pos": [],
                "joint_vel": [],
                "finger_pos": [],
                "finger_vel": [],
                "gripper_width": [],
                "eef_pos": [],
                "eef_vel": [],
                "eef_quat": [],
                "eef_quat_vel": [],
                "des_joint_pos" : [],
                "des_joint_vel" : [],
                "des_joint_acc" : [],
                "des_finger_pos" : [],
                "des_eef_pos": [],
                "des_eef_vel": [],
                "des_eef_quat": [],
                "des_eef_quat_vel": [],
                "time_stamp": [],
            }


        self.log_counter = 0
        self.save_path = os.path.join(
            self.save_root_path,
            "{}_{}".format(task_type, datetime.now().strftime("%Y_%m_%d_%H_%M_%S")),
        )
        
        self.ab_path = os.path.abspath(self.save_path)
        print(f"Save record absolute path initialized: {self.ab_path}")
        # flags
        self.record_mode = False
        self.manager = manager

    def start_record(self):
        # if not self.record_mode :
        #     return
        print("Start recording")
        self.record_mode = True
    

    def stop_record(self):
        # if not self.record_mode :
        #     return
        print("Stop recording")
        self.record_mode=False
        
    def update_obj_log(self):
        
        for obj in self.obj_list:
            obj_name=obj.name
            self.obj_log_dict[obj_name]["pos"].append(
                self.scene.get_obj_pos(obj).flatten()
            )
            self.obj_log_dict[obj_name]["orientation"].append(
                self.scene.get_obj_quat(obj).flatten()
            )
            self.obj_log_dict[obj_name]["time_stamp"].append(
                self.scene.time_stamp
            )
    def update_robot_log(self):
        for robot in self.robots:              
            self.robot_log_dict[robot]["joint_pos"].append(self.robot.current_j_pos),
            self.robot_log_dict[robot]["joint_vel"].append(self.robot.current_j_vel),
            self.robot_log_dict[robot]["finger_pos"].append(self.robot.current_fing_pos),
            self.robot_log_dict[robot]["finger_vel"].append(self.robot.current_fing_vel),
            self.robot_log_dict[robot]["gripper_width"].append(self.robot.gripper_width),
            self.robot_log_dict[robot]["eef_pos"].append(self.robot.current_c_pos),
            self.robot_log_dict[robot]["eef_vel"].append(self.robot.current_c_vel),
            self.robot_log_dict[robot]["eef_quat"].append(self.robot.current_c_quat),
            self.robot_log_dict[robot]["eef_quat_vel"].append(self.robot.current_c_quat_vel),
            self.robot_log_dict[robot]["des_joint_pos"].append(self.robot.des_joint_pos),
            self.robot_log_dict[robot]["des_joint_vel"].append(self.robot.des_joint_vel),
            self.robot_log_dict[robot]["des_joint_acc"].append(self.robot.des_joint_acc),
            self.robot_log_dict[robot]["des_finger_pos"].append(self.robot.set_gripper_width),
            self.robot_log_dict[robot]["des_c_pos"].append(self.robot.des_c_pos),
            self.robot_log_dict[robot]["des_c_vel"].append(self.robot.des_c_vel),
            self.robot_log_dict[robot]["des_quat"].append(self.robot.des_quat),
            self.robot_log_dict[robot]["des_quat_vel"].append(self.robot.des_quat_vel),
            self.robot_log_dict[robot]["time_stamp"].append(
                    self.scene.time_stamp
                )
            
        




    def save_record(self):
        if not self.record_mode:
            return
        self.stop_record()
        file_name = "{}_{:03d}.pkl".format(self.task_type, self.log_counter)
        self.log_counter += 1
        mp.Process(target=self._save_record, kwargs={"file_name": file_name}).start()

    def _save_record(self, file_name):
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        state_dict = dict()
        for robot, robot_log_data in self.robot_log_dict():
            # state_dict["robot"] = robot.robot_logger.log_dict_full
            robot_state_dict = dict()
            # robot_log_data = robot.robot_logger.log_dict_full
            
            for attr, data in robot_log_data.items():
                robot_state_dict[attr] = data[:: self.downsample_steps]
            state_dict[robot] = robot_state_dict

        for obj_name, obj_log_data in self.obj_log_dict():
            # state_dict[obj_name] = obj_logger.log_dict_full
            obj_state_dict = dict()
            print(f"this is recorder obj:{obj_state_dict}")
            for attr, data in obj_log_data.items():
                obj_state_dict[attr] = data[:: self.downsample_steps]
            state_dict[obj_name] = obj_state_dict
            