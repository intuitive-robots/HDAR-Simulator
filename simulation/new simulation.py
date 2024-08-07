# import torch
import time
import numpy as np
from typing import List
from enum import Flag, auto

import os
from datetime import datetime
import multiprocessing as mp

from alr_sim.core import Scene, RobotBase
from alr_sim.controllers.Controller import ControllerBase
from alr_sim.sims.SimFactory import SimRepository
from simpub.sim.sf_publisher import SFPublisher
import server, controllers, utils, tasks
import mujoco
import math
from .collision_finger import Collision_finger
from simpub.xr_device.meta_quest3 import MetaQuest3




class SimFlag(Flag):
    """
    - shutdown
    - running
        - waiting for server
        - connected
            - on stream
            - wait for reset
            - resetting
    """

    SHUTDOWN = auto()
    WAITING_FOR_CONNECTION = auto()
    ON_TASK = auto()
    WAITING_FOR_RESET = auto()
    # RESETTING = auto()
    RESETTASK0, RESETTASK1, RESETTASK2, RESETTASK3 = auto(), auto(), auto(), auto()
    RESETTING = RESETTASK0 | RESETTASK1 | RESETTASK2 | RESETTASK3

    CONNECTED = ON_TASK | WAITING_FOR_RESET | RESETTING
    RUNNING = WAITING_FOR_CONNECTION | CONNECTED


class Simulation:
    def __init__(
        self,
        task_type,
        record_mode=False,
        time_limit=1000000000,
        dt=0.002,
        downsample_steps=1        
    ) -> None:

        self.vt_factory = SimRepository.get_factory("mj_beta")

        self.time_limit = time_limit
        self.task_type = task_type
        self.record_mode = record_mode
        self.downsample_steps = downsample_steps
        self.dt = dt

        self.replace_rb0_l=0
        self.replace_rb0_r=0
        self.replace_rb1_l=0
        self.replace_rb1_r=0
      
        
        # virtual twin
        #task manage
        self.task_manager = tasks.get_manager(
            task_type, self.vt_factory, self.dt
        )
        self.task_manager.create_task()
        #scene and object
        self.vt_scene: Scene = self.task_manager.get_scene()
        self.vt_object_list = self.task_manager.get_object_list()
        self.vt_robot_dict = self.task_manager.get_robot_dict()
        for k, v in self.vt_robot_dict.items():
            print(k, v)
        # self.setup_controllers()
        self.start_scenes() 
        self.setup_callbacks()
        self.setup_publisher()
        # self.publisher()
        self.set_controllers()
        self.start_controllers()
        # self.setup_streamer()
        # self.streamer.start()
        self.set_recorder()
        self.setup_cli()
        self.cli.start()

        self.current_time = 0
        self.status: SimFlag = SimFlag.RUNNING
# #rb0
        self.rb0_finger_collision = Collision_finger(
            self.vt_scene,
            target_pairs1={('target:geom', 'finger1_rb0_tip_collision')},
            target_pairs2={('target:geom', 'finger2_rb0_tip_collision')}
        )
#rb1
        self.rb1_finger_collision = Collision_finger(
            self.vt_scene,
            target_pairs1={('target:geom', 'finger1_rb1_tip_collision')},
            target_pairs2={('target:geom', 'finger2_rb1_tip_collision')}
        )
        

        self.force_interval = 0.02
        self.force_last_timestep = -self.force_interval

    def setup_callbacks(self):
        self.reset_flag = False
        self.start_record_flag = False
        self.stop_record_flag = False
        self.save_record_flag = False
        self.change_task_flag = None
        self.vt_scene.register_callback(self._reset_clb)
        # self.vt_scene.register_callback(self._start_record_clb)
        # self.vt_scene.register_callback(self._stop_record_clb)
        # self.vt_scene.register_callback(self._save_record_clb)
        self.vt_scene.register_callback(self._change_task_clb)
        self.vt_scene.register_callback(self._max_time_clb)
        self.vt_scene.register_callback(self._task_finished_clb)

    def setup_cli(self):
        # GLI setting
        self.cli = utils.CLI()
        self.cli.register_function("Q", "close", self.shutdown_cli)
        self.cli.register_function("R", "Reset", self.reset)
        # self.cli.register_function("OG", "Open Gripper", self.open_grippers)
        # self.cli.register_function("CG", "Close Gripper", self.close_grippers)
        # self.cli.register_function("DR", "Start Record", self.start_record)
        # self.cli.register_function("SR", "Stop Record", self.stop_record)
        # self.cli.register_function("L", "Save and Reset", self.save_and_reset)
        # self.cli.register_function("SQR", "Start QR Teleport", self.streamer.start_qr_teleport)
        # self.cli.register_function("CQR", "Close QR Teleport", self.streamer.close_qr_teleport)
        # self.cli.register_function(
        #     "MT", "Set Objects Manipulable True", self.streamer.set_objects_manipulable_true
        # )
        # self.cli.register_function(
        #     "MF", "Set Objects Manipulable False", self.streamer.set_objects_manipulable_false
        # )
        # self.cli.register_function(
        #     "AEER",
        #     "Activate End Effector Recorder",
        #     self.streamer.activate_end_effector_recorder,
        # )
        # self.cli.register_function(
        #     "DEER",
        #     "Deactivate End Effector Recorder",
        #     self.streamer.deactivate_end_effector_recorder,
        # )
        self.cli.register_function("0", "Reset Task 0", self.change2task("warm_up"))
        self.cli.register_function(
            "1", "Reset Task 1", self.change2task("box_stacking")
        )
        self.cli.register_function(
            "2", "Reset Task 2", self.change2task("cup_stacking")
        )
        self.cli.register_function(
            "3", "Reset Task 3", self.change2task("practical_manipulation")
        )

    # def setup_recorder(self):
    #     self.recorder = server.UnityRecorder(
    #         self.vt_scene,
    #         self.vt_object_list,
    #         self.task_type,
    #         # self.streamer,
    #         manager=self.task_manager,
    #         record_mode=self.record_mode,
    #         downsample_steps=self.downsample_steps,
    #     )
    def set_recorder(self):
        self.save_root_path="./ARHumanDemoData/"
        self.obj_log_dict = dict()
        self.robot_log_dict = dict()
        self.log_counter = 0
        for obj in self.vt_object_list:
            self.obj_log_dict[obj.name] = {
                "pos": [],
                "orientation": [],
                "time_stamp": []
            }
        
        for robot in self.vt_robot_dict:
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
        self.save_path = os.path.join(
            self.save_root_path,
            "{}_{}".format(self.task_type, datetime.now().strftime("%Y_%m_%d_%H_%M_%S")),
        )
        self.ab_path = os.path.abspath(self.save_path)
        print(f"Save record absolute path initialized: {self.ab_path}")

        


    def obj_recorder(self):
        for obj in self.vt_object_list:
            self.obj_log_dict[obj.name]["pos"].append(
                self.vt_scene.get_obj_pos(obj)
            )
            self.obj_log_dict[obj.name]["orientation"].append(
                self.vt_scene.get_obj_quat(obj).flatten()
            )
            self.obj_log_dict[obj.name]["time_stamp"].append(
                self.vt_scene.time_stamp
            )
    def robot_recorder(self):
        self.robots: RobotBase = self.vt_scene.robots
        for name, robot in self.vt_robot_dict.items():
            self.robot_log_dict[name]["joint_pos"].append(robot.current_j_pos),
            self.robot_log_dict[name]["joint_vel"].append(robot.current_j_vel),
            self.robot_log_dict[name]["finger_pos"].append(robot.current_fing_pos),
            self.robot_log_dict[name]["finger_vel"].append(robot.current_fing_vel),
            self.robot_log_dict[name]["gripper_width"].append(robot.gripper_width),
            self.robot_log_dict[name]["eef_pos"].append(robot.current_c_pos),
            self.robot_log_dict[name]["eef_vel"].append(robot.current_c_vel),
            self.robot_log_dict[name]["eef_quat"].append(robot.current_c_quat),
            self.robot_log_dict[name]["eef_quat_vel"].append(robot.current_c_quat_vel),
            self.robot_log_dict[name]["des_joint_pos"].append(robot.des_joint_pos),
            self.robot_log_dict[name]["des_joint_vel"].append(robot.des_joint_vel),
            self.robot_log_dict[name]["des_joint_acc"].append(robot.des_joint_acc),
            self.robot_log_dict[name]["des_finger_pos"].append(robot.set_gripper_width),
            self.robot_log_dict[name]["des_eef_pos"].append(robot.des_c_pos),
            self.robot_log_dict[name]["des_eef_vel"].append(robot.des_c_vel),
            self.robot_log_dict[name]["des_eef_quat"].append(robot.des_quat),
            self.robot_log_dict[name]["des_eef_quat_vel"].append(robot.des_quat_vel),
            self.robot_log_dict[name]["time_stamp"].append(
                    self.vt_scene.time_stamp
                )
            
    def _save_record(self):
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        state_dict = dict()
        for robot, robot_log_data in self.robot_log_dict.items():
            # state_dict["robot"] = robot.robot_logger.log_dict_full
            robot_state_dict = dict()
            # robot_log_data = robot.robot_logger.log_dict_full
            
            for attr, data in robot_log_data.items():
                robot_state_dict[attr] = data[:: self.downsample_steps]
            state_dict[robot] = robot_state_dict

        for obj_name, obj_log_data in self.obj_log_dict.items():
            # state_dict[obj_name] = obj_logger.log_dict_full
            obj_state_dict = dict()
            print(f"this is recorder obj:{obj_state_dict}")
            for attr, data in obj_log_data.items():
                obj_state_dict[attr] = data[:: self.downsample_steps]
            state_dict[obj_name] = obj_state_dict
    def save_record(self):
        
        if not self.record_mode:
            return
        file_name = "{}_{:03d}.pkl".format(self.task_type, self.log_counter)
        self.log_counter += 1
        mp.Process(target=self._save_record, kwargs={"file_name": file_name}).start()


    def setup_publisher(self):
        hdar_config = utils.get_hdar_config()    #host address
        self.publisher = SFPublisher(        
            self.vt_scene,
            **hdar_config,
            no_tracked_objects=[]
        )

        return self.publisher

    # def setup_streamer(self):
    #     hdar_config = utils.get_hdar_config()    #host address
    #     self.streamer = SFPublisher(
    #         self.vt_scene,
    #         self.vt_robot_dict.values(),
    #         self.vt_object_list,
    #         **hdar_config,
    #     )
    #     self.streamer.register_callback(start_record=self.start_record)
    #     self.streamer.register_callback(stop_record=self.stop_record)
    #     self.streamer.register_callback(reset=self.reset)
    #     self.streamer.register_callback(save_and_reset=self.save_and_reset)
    #     self.streamer.register_callback(open_grippers=self.open_grippers)
    #     self.streamer.register_callback(close_grippers=self.close_grippers)

    # def setup_controllers(self):
    #     self.controller_list: List[ControllerBase] = list()
    #     self.robot_list: List[RobotBase] = list()
    #     self.real_robot_list: List[poly_controllers.Panda] = list()
    #     # self.scene_list: List[Scene] = [self.vt_scene]

    #     for robot_name, robot_config in self.task_manager.robots_config.items():
    #         # for vt_robot_handler in self.streamer.robot_handlers:
    #         vt_robot = self.vt_robot_dict[robot_name]
    #         interaction_method = vt_robot.interaction_method
    #         self.interaction_method = str(interaction_method)
    #         real_robot = None 
    #         if interaction_method == "real_robot":
    #             # continue
    #             # set up real scene
    #             real_robot_config = utils.get_real_robot_config()[robot_name]
    #             real_robot = poly_controllers.Panda(
    #                 name=robot_name,
    #                 **real_robot_config,
    #             )
    #             real_controller = controllers.RealRobotController(real_robot)
    #             self.real_robot_list.append(real_robot)

    #             vt_controller = controllers.VTController(
    #                 vt_robot,
    #                 real_robot,
    #                 self.vt_scene,
    #                 robot_config,
    #             )
    #             self.controller_list.append(vt_controller)
    #             self.robot_list.append(vt_robot)

    #         else:
    #             if interaction_method == "Meta_Controller_r0":
    #                 controller_cls = controllers.MetaQuest3Controller_right
    #                 self.robot_list.append(vt_robot)
                    
    #             elif interaction_method == "Meta_Controller_l0":
    #                 controller_cls = controllers.MetaQuest3Controller_left
    #                 self.robot_list.append(vt_robot)
                    

    #             else: 

    #                 if interaction_method == "virtual_robot":
    #                     controller_cls = controllers.VirtualRobotTCPController
    #                 elif interaction_method == "hand_tracking":
    #                     controller_cls = controllers.HandTrackerTCPController
    #                 elif interaction_method == "keyboard":
    #                     controller_cls = controllers.KeyboardTCPController
    #                 elif interaction_method == "motion_controller":
    #                     controller_cls = controllers.ViveProMotionControllerTCPController
            
            
    #             vt_controller = controller_cls(
    #                 self.vt_scene,
    #                 vt_robot,
    #                 robot_config,
    #             )

    #             self.controller_list.append(vt_controller)
    #             print(f"self.controller_list:{self.controller_list}")
    #             self.robot_list.append(vt_robot)
    #             print(f"self.robot_list:{self.robot_list}")
    #             for robot, controller in zip(self.robot_list, self.controller_list):
    #                 print(f"i am robot:{robot.time_stamp}")
    #                 print(f"robot type: {type(robot)}")  
                             

    def start_scenes(self):
        self.scene_list: List[Scene] = [self.vt_scene]
        for scene in self.scene_list:
            scene.start()
    
    def set_controllers(self):
        self.controller_list: List[ControllerBase] = list()
        self.robot_list: List[RobotBase] = list()
        self.real_robot_list: List[poly_controllers.Panda] = list()
        self.scene_list: List[Scene] = [self.vt_scene]
        meta_quest3 = MetaQuest3("ALRMetaQuest3")
        self.meta_quest3 = meta_quest3 

        for robot_name, robot_config in self.task_manager.robots_config.items():
            # for vt_robot_handler in self.streamer.robot_handlers:
            vt_robot = self.vt_robot_dict[robot_name]
            interaction_method = vt_robot.interaction_method
            self.interaction_method = str(interaction_method)
            # if interaction_method.get("device") == "meta1_alr1":
            #         meta_address="192.168.0.102"
            # elif interaction_method.get("device") == "meta1_alr2":
            #         meta_address="192.168.0.143"
            real_robot = None 
            if interaction_method == "real_robot":
                continue
                # set up real scene
                real_robot_config = utils.get_real_robot_config()[robot_name]
                real_robot = poly_controllers.Panda(
                    name=robot_name,
                    **real_robot_config,
                )
                real_controller = controllers.RealRobotController(real_robot)
                self.real_robot_list.append(real_robot)

                vt_controller = controllers.VTController(
                    vt_robot,
                    real_robot,
                    self.vt_scene,
                    robot_config,
                )

            else:
                

                if interaction_method.get("type") == "Meta_Controller_r0":
                    controller_cls = controllers.MetaQuest3Controller_right
                    vt_controller = controller_cls(
                        meta_quest3
                    )

                elif interaction_method.get("type") == "Meta_Controller_l0":
                    controller_cls = controllers.MetaQuest3Controller_left
                    vt_controller = controller_cls(
                        meta_quest3
                    )
                
            self.controller_list.append(vt_controller)
            print(f"i am controller_list:{self.controller_list}") 
            self.robot_list.append(vt_robot)
        return self.meta_quest3

    def start_controllers(self):
       
        for robot, controller in zip(self.robot_list, self.controller_list):
            if isinstance(controller, controllers.VTController):
                robot.beam_to_joint_pos(
                    controller.real_robot.robot.get_joint_positions().numpy()
            )
            print(f"self.robot_list:{robot}")
            print(f"robot type: {type(robot)}")
            print(f"i am controller:{controller}") 
            controller.executeController(robot, maxDuration=1000, block=False)   
    
    def change2task(self, task_type):
        def f():
            self.change_task_flag = task_type

        return f

    def _change_task_clb(self):
        if self.change_task_flag is not None:
            task_type = self.change_task_flag
            self.change_task_flag = None
            if hasattr(self.task_manager, "change2task"):
                self.task_manager.change2task(task_type)

    def reset(self):
        self.stop_record_flag = True
        self.reset_flag = True

    def _reset_clb(self):
        if self.reset_flag:
            self.reset_flag = False
            self.reset_initial_pose()

    # def start_record(self):
    #     self.start_record_flag = True

    # def _start_record_clb(self):
    #     if self.start_record_flag:
    #         self.start_record_flag = False
    #         self.stop_record_flag = False
    #         self.recorder.start_record()

    # def stop_record(self):
    #     self.stop_record_flag = True

    # def _stop_record_clb(self):
    #     if self.stop_record_flag:
    #         self.stop_record_flag = False
    #         self.recorder.stop_record()

    def save_and_reset(self):
        self.save_record_flag = True
        self.reset_flag = True

    def _save_record_clb(self):
        if self.save_record_flag:
            self.save_record_flag = False
            self.recorder.save_record()

    def _max_time_clb(self):
        return
        if self.recorder.on_logging:
            if self.current_time < self.time_limit:
                self.current_time += 1
            else:
                self.current_time = 0
                print("Time is up")
                self.stop_record_flag = True

    def _task_finished_clb(self):
        if self.task_manager.is_task_finished() and self.status is SimFlag.ON_TASK:
            self.stop_record_flag = True
            # self.streamer.send_message("task_finished")
            self.status = SimFlag.WAITING_FOR_RESET

    def open_grippers(self, *args, robot_name="panda1", **kwargs):
        self.vt_robot_dict[robot_name].open_fingers()

    def close_grippers(self, *args, robot_name="panda1", **kwargs):
        self.vt_robot_dict[robot_name].close_fingers(duration=0)

    def terminate_policies(self):
        for real_robot in self.real_robot_list:
            if real_robot.is_running_policy():
                real_robot.robot.terminate_current_policy()

    def shutdown_cli(self, *args, **kwargs):
        self.recorder.stop_record()
        # self.streamer.close_server()
        for controller in self.controller_list:
            controller._max_duration = 0
        self.terminate_policies()
        self.status = SimFlag.SHUTDOWN
        # self.streamer.shutdown()
        self.publisher.shutdown()
        return False

    def reset_initial_pose(self):
        self.status = SimFlag.RESETTING
        self.task_manager.reset_objects()
        self.task_manager.reset_robots()
        # self.streamer.send_message("new_task_ready")
        self.status = SimFlag.ON_TASK

    def update_force_feedback(self):
        if (
            self.scene_list[0].time_stamp
            > self.force_last_timestep + self.force_interval
        ):
            self.force_last_timestep = self.scene_list[0].time_stamp
            for vt_robot, real_robot in zip(self.robot_list, self.real_robot_list):
                if real_robot.is_running_policy():
                    constraint_forces = [
                        self.vt_scene.data.joint(name).qfrc_constraint[0]
                        for name in vt_robot.joint_names
                    ]
                    constraint_forces = torch.tensor(np.array(constraint_forces))
                    try:
                        real_robot.update_constraint_forces(constraint_forces)
                    except:
                        pass

#collision force ro0 update

# #collision force for left finger
#     def check_collision_finger1(self):
#         collision_finger1_list=[]    
#         filename1 = 'collision_finger1.txt'  
#         target_pairs1 = {('target:geom', 'finger1_rb0_tip_collision')}
#         #target_geom_ids = [self.model.geom_name2id(name) for name in target_geoms]
#         tt=range(self.vt_scene.data.ncon)
#         for i in range(self.vt_scene.data.ncon):

#             contact = self.vt_scene.data.contact[i]    
#             pos = np.array(contact.pos[:3])
#             geom1_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
#             geom2_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
#             force = np.zeros(6)
#             mujoco.mj_contactForce(self.vt_scene.model, self.vt_scene.data, i, force)
#             time=self.vt_scene.data.time
#             #if geom1_name == "finger1_rb0_tip_collision" or geom2_name == "finger1_rb0_tip_collision":
#             if (geom1_name, geom2_name) in target_pairs1 or (geom2_name, geom1_name) in target_pairs1:
#                 collision_finger1_list.append({
#                 'geom1': geom1_name,
#                 'geom2': geom2_name,
#                 'position': pos.astype(float),
#                 'force': force[:3].astype(float), # Only take the first three elements (force)
#                 'time': time,
#                 'number': i
#                 })
#                 with open(filename1, 'a', newline='') as file:
#                     file.write(f"Collision object:{geom1_name} with {geom2_name} at {pos}, Force: {force[:3]}, time:{time},number:{i}:{tt}\n")
#         return collision_finger1_list
# #collision resultant force for left finger
#     def finger1_resultant_force(self):
#         collision_finger1_list = self.check_collision_finger1()
#         collision_finger_resultant = {}
#         replace=0
#         for collision in collision_finger1_list:
#             geom1=collision['geom1']
#             geom2=collision['geom2']
#             pos=collision['position']
#             force=collision['force']
#             time=collision['time']
#             time_key=round(collision['time'],4)
#             geom_pair=tuple(sorted((geom1, geom2)))
#             if (time_key, geom_pair) not in collision_finger_resultant:
#                 collision_finger_resultant[(time_key, geom_pair)] = {
#                     'geom1': geom1,
#                     'geom2': geom2,
#                     'position_sum': pos,
#                     'force': force,
#                     'newtime': time_key,
#                     'number': 1
#                 }
#             else:
#                 collision_finger_resultant[(time_key, geom_pair)]['force'] += force
#                 collision_finger_resultant[(time_key, geom_pair)]['position_sum'] += pos
#                 collision_finger_resultant[(time_key, geom_pair)]['number'] += 1
#         collision_finger1_resultant_result = []
#         for (time_key, geom_pair),data in collision_finger_resultant.items():
#             avg_position = data['position_sum'] / data['number']
#             collision_finger1_resultant_result.append({
#                 'geom1': geom1,
#                 'geom2': geom2,
#                 'position': avg_position,
#                 'force': force,
#                 'time': time
#             })
                        
#             force_x, force_y, force_z = force
#             resultantforce=math.sqrt(force_x**2 + force_y**2 + force_z**2)
            
#            #replace force to 0-1,in the userstudy threshold:10
#             if resultantforce >= 10:
#                 replace=1,
#             else:
#                 normalized_value = resultantforce / 10
#                 replace= np.sqrt(normalized_value)
           
#             filename3 = 'collision_finger1_resultant.txt'
            
#             self.replace_rb0_l=replace
#             with open(filename3, 'a', newline='') as file:
#                  file.write(f"Collision object:{geom1} with {geom2} at {avg_position}, Force: {force}{replace}, time:{time}\n")
            
#             # return collision_finger1_resultant_result, self.replace_rb0_l
#             return self.replace_rb0_l
    
# #collision force for right finger
#     def check_collision_finger2(self):
#         collision_finger2_list=[]    
#         filename1 = 'collision_finger2.txt'  
#         target_pairs1 = { ('target:geom', 'finger2_rb0_tip_collision')}
#         #target_geom_ids = [self.model.geom_name2id(name) for name in target_geoms]
#         tt=range(self.vt_scene.data.ncon)
#         for i in range(self.vt_scene.data.ncon):

#             contact = self.vt_scene.data.contact[i]    
#             pos = np.array(contact.pos[:3])
#             geom1_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
#             geom2_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
#             force = np.zeros(6)
#             mujoco.mj_contactForce(self.vt_scene.model, self.vt_scene.data, i, force)
#             time=self.vt_scene.data.time
#             #if geom1_name == "finger1_rb0_tip_collision" or geom2_name == "finger1_rb0_tip_collision":
#             if (geom1_name, geom2_name) in target_pairs1 or (geom2_name, geom1_name) in target_pairs1:
#                 collision_finger2_list.append({
#                 'geom1': geom1_name,
#                 'geom2': geom2_name,
#                 'position': pos.astype(float),
#                 'force': force[:3].astype(float), # Only take the first three elements (force)
#                 'time': time,
#                 'number': i
#                 })
#                 # with open(filename1, 'a', newline='') as file:
#                 #     file.write(f"Collision object:{geom1_name} with {geom2_name} at {pos}, Force: {force[:3]}, time:{time},number:{i}:{tt}\n")
#         return collision_finger2_list
# #collision resultant force for right finger
#     def finger2_resultant_force(self):
#         collision_finger2_list = self.check_collision_finger2()
#         collision_finger_resultant = {}
#         replace=0
#         for collision in collision_finger2_list:
#             geom1=collision['geom1']
#             geom2=collision['geom2']
#             pos=collision['position']
#             force=collision['force']
#             time=collision['time']
#             time_key=round(collision['time'],4)
#             geom_pair=tuple(sorted((geom1, geom2)))
#             if (time_key, geom_pair) not in collision_finger_resultant:
#                 collision_finger_resultant[(time_key, geom_pair)] = {
#                     'geom1': geom1,
#                     'geom2': geom2,
#                     'position_sum': pos,
#                     'force': force,
#                     'newtime': time_key,
#                     'number': 1
#                 }
#             else: 
#                 collision_finger_resultant[(time_key, geom_pair)]['force'] += force
#                 collision_finger_resultant[(time_key, geom_pair)]['position_sum'] += pos
#                 collision_finger_resultant[(time_key, geom_pair)]['number'] += 1
#         collision_finger2_resultant_result = []
#         for (time_key, geom_pair),data in collision_finger_resultant.items():
#             avg_position = data['position_sum'] / data['number']
#             collision_finger2_resultant_result.append({
#                 'geom1': geom1,
#                 'geom2': geom2,
#                 'position': avg_position,
#                 'force': force,
#                 'time': time
#             })
             
#             force_x, force_y, force_z = force
#             resultantforce=math.sqrt(force_x**2 + force_y**2 + force_z**2)
#                   #replace force to 0-1,in the userstudy threshold:10
#             if resultantforce >= 10:
#                 replace=1,
#             else:
#                 normalized_value = resultantforce / 10
#                 replace= np.sqrt(normalized_value)
#             filename3 = 'collision_finger2_resultant.txt'
#             self.replace_rb0_r=replace
#             # with open(filename3, 'a', newline='') as file:
#             #      file.write(f"Collision object:{geom1} with {geom2} at {avg_position}, Force: {force}{self.replace_rb0_r}, time:{time}\n")
            
#             # return collision_finger2_resultant_result, self.replace_rb0_r
#             return self.replace_rb0_r
    
#     # def rb0_finger_collision(self):
#     #     rb0fingertotalrepalce=(np.array(self.replace_rb0_l)+np.array(self.replace_rb0_r)) / 2
#     #     return rb0fingertotalrepalce
    

#collision force for aim
    def check_collision_aim(self):
        collision_aim_list=[]
            
        filename2 = 'collision_aim.txt'  
        target_pairs2={('target:geom', 'aim:geom'),('target:geom', 'aim_1:geom'),('target:geom', 'aim_2:geom'),('target:geom', 'aim_3:geom'),('target:geom', 'aim_4:geom')}
        #target_geom_ids = [self.model.geom_name2id(name) for name in target_geoms]
        tt=range(self.vt_scene.data.ncon)
        for i in range(self.vt_scene.data.ncon):
            contact = self.vt_scene.data.contact[i]    
            pos = np.array(contact.pos[:3])
            geom1_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            force = np.zeros(6)
            mujoco.mj_contactForce(self.vt_scene.model, self.vt_scene.data, i, force)
            time=self.vt_scene.data.time
            #if geom1_name == "aim:geom" or geom2_name == "aim:geom":
            if (geom1_name, geom2_name) in target_pairs2 or (geom2_name, geom1_name) in target_pairs2:
                collision_aim_list.append({
                'geom1': geom1_name,
                'geom2': geom2_name,
                'position': pos.astype(float),
                'force': force[:3].astype(float), # Only take the first three elements (force)
                'time': time,
                'number': i
                })
                # with open(filename2, 'a', newline='') as file:
                #      file.write(f"Collision object:{geom1_name} with {geom2_name} at {pos}, Force: {force[:3]}, time:{time},number:{i}:{tt}\n")
        return collision_aim_list
    
    def aim_resultant_force(self):
        collision_aim_list = self.check_collision_aim()
        collision_aim_resultant={}
        filename4 = 'collision_aim_resultant.txt'
        replace=0
            
        for collision in collision_aim_list:
            geom1=collision['geom1']
            geom2=collision['geom2']
            pos=collision['position']
            force=collision['force']
            time=collision['time']
            time_key=round(collision['time'],4)
            geom_pair=tuple(sorted((geom1, geom2)))
                
            if (time_key,geom_pair) not in collision_aim_resultant:
                collision_aim_resultant[(time_key, geom_pair)] = {
                        'geom1': geom1,
                        'geom2': geom2,
                        'position_sum': pos,
                        'force': force,
                        'newtime': time_key,
                        'number': 1
                }
            else:
                collision_aim_resultant[(time_key,geom_pair)]['force'] += force[:3]
                collision_aim_resultant[(time_key,geom_pair)]['position_sum'] += pos
                collision_aim_resultant[(time_key,geom_pair)]['number'] += 1
                
        collision_aim_resultant_result=[]
        for (time_key,geom_pair), data in collision_aim_resultant.items():
            avg_position = data['position_sum'] / data['number']
            collision_aim_resultant_result.append({
                'geom1': geom1,
                'geom2': geom2,
                'position': avg_position,
                'force': force,
                'time': time
            })
            force_x, force_y, force_z = force
            resultantforce=math.sqrt(force_x**2 + force_y**2 + force_z**2)
            replaceaim=0
            #replace force to 0-1,in the userstudy threshold:10
            if  self.replace_rb0_l != 0 or self.replace_rb0_r != 0 or self.replace_rb1_l != 0 or self.replace_rb1_r != 0:
                if resultantforce >= 10:
                    replace=1,
                else:
                    normalized_value = resultantforce / 10
                    replace= np.sqrt(normalized_value)
                replaceaim=replace
                # with open(filename4, 'a', newline='') as file:
                #     file.write(f"Collision object:{geom1} with {geom2} at {avg_position}, Force: {force}{replaceaim}, time:{time}\n")
                            
                return replaceaim



      
    def run(self):
        self.terminate_policies()
        self.reset_initial_pose()
        print('prepaired')
        steps = 0
        start = time.time()
        collision_time_interval=0.01   #interval for collision data collection 0.01s
        last_check_time=time.time()
        # r = recorder(setup)
        while self.status in SimFlag.RUNNING:
            while steps * self.dt > time.time() - start:
                pass
            steps += 1
            self.update_force_feedback()
           #check collision and output rb0
            current_time =time.time()
            if current_time - last_check_time >= collision_time_interval:
                # self.check_collision_finger1()
                # self.check_collision_finger2()
                # self.finger1_resultant_force()
                # self.finger2_resultant_force()
                # self.rb0_finger_collision()
                
               #collision rb1
                replace_rb0_l=0
                replace_rb0_r=0
                replace_rb0_l, replace_rb0_r = self.rb0_finger_collision.get_collisions()
               
                #collision rb1
                replace_rb1_l, replace_rb1_r = self.rb1_finger_collision.get_collisions()
                rb0_finger_collision=0
                # print(f"r0_finger_l:{replace_rb0_l}")
                rb0_finger_collision = (np.array(replace_rb0_l) + np.array(replace_rb0_r)) / 2
                rb1_finger_collision = (np.array(replace_rb1_l) + np.array(replace_rb1_r)) / 2
                if rb0_finger_collision != 0:
                    self.meta_quest3.publish_vibrate()
                if rb1_finger_collision != 0:
                    self.meta_quest3.publish_vibrate()
                    # print(f"Leftrobot Left Finger Value: {replace_rb1_l}")
                    # print(f"Leftrobot Right Finger Value: {replace_rb1_r}")
                # print(f"RB0 Finger Collision Average Replace Value: {rb0_finger_collision},RB1 Finger Collision Average Replace Value: {rb1_finger_collision}")
                self.check_collision_aim()
                # replaceaim=self. aim_resultant_force()
                # if replaceaim != 0:
                #     self.meta_quest3.publish_vibrate()
                # #vibraton publish
                # if rb0_finger_collision != 0:
                #     if 

                last_check_time = current_time
                
            record_num=0
  
            for scene in self.scene_list:
                scene.next_step()

                if record_num %0.1 == 0:
                    self.obj_recorder()
                    self.robot_recorder()
                    # print(self.robot_log_dict)
                    # self._save_record()
                    self.save_record()
                    record_num += 1


            # if record_num %10 == 0:
            #     r.record(object)
            #     r.record(robot)
            #     r.record(interaction)
            #     record_num += 1
                         
        print("Goodbye")
                
