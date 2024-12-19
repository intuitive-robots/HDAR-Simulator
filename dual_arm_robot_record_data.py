import argparse
import yaml
import torch
from hdar_simulator._simulationframework import sf_task_factory
from hdar_simulator._simulationframework import SFRecorder
from hdar_simulator._simulationframework import MetaQuest3Controller
from hdar_simulator._simulationframework import SFSimulator
from simpub.xr_device.meta_quest3 import MetaQuest3
import numpy as np
import poly_controllers,controllers
from typing import List
from alr_sim.core import RobotBase
from alr_sim.controllers.Controller import ControllerBase

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", default='BoxPushing'
    )
    parser.add_argument(
        "--host", default='192.168.0.143'
    )
    parser.add_argument("-i", default='meta_quest3')
    # parser.add_argument("-i", default='real_robot')
    args = parser.parse_args()

    # simulator: SFSimulator = sf_task_factory(args.t, host_address=args.host)
    # recorder = SFRecorder(simulator)
    # controller_config = yaml.safe_load(
    #     open(
    #         "./hdar_simulator/_simulationframework/task/" +
    #         f"{args.t}/controller.yaml", "r"
    #     )
    # )
    if args.i == 'meta_quest3':
        modified_task_type = args.t + "Vibration"
        simulator: SFSimulator = sf_task_factory(modified_task_type, host_address=args.host)
        recorder = SFRecorder(simulator)
        controller_config = yaml.safe_load(
            open(
                "./hdar_simulator/_simulationframework/task/" +
                f"{args.t}/controller.yaml", "r"
            )
        )
        meta_quest3 = MetaQuest3("ALR2")
        meta_quest3.register_button_press_event("X", recorder.save_record)
        meta_quest3.register_button_press_event(

            "X", simulator.reset_in_the_main_thread
        )

        meta_controller1 = MetaQuest3Controller(
            meta_quest3,
            hand="right",
            **controller_config["meta_quest3"],
        )
        meta_controller2 = MetaQuest3Controller(
            meta_quest3,
            hand="left",
            **controller_config["meta_quest3"],
        )
        meta_quest3.register_trigger_press_event(
            "hand_trigger", "right", recorder.start_record
        )
        meta_quest3.register_trigger_release_event(
            "hand_trigger", "right", recorder.stop_record
        )
        simulator.assign_controller({"panda_robot1": meta_controller1 , "panda_robot2": meta_controller2})
        vibration_detect = False
        vibration_detect_left = False
        vibration_aim_detect = False
        simulator.reset()
        while True:
            simulator.after_step()
            input_data=meta_quest3.get_input_data()
            if simulator.haptic_on_right :
                hand_right = input_data["right"]
                
                if hand_right["index_trigger"] and not vibration_detect:
                    meta_quest3.start_vibration(duration=0.15)
                    vibration_detect = True
                    
                elif not hand_right["index_trigger"] :
                    meta_quest3.start_vibration(duration=0.1)
                    vibration_detect = True
                    
                elif hand_right["index_trigger"] and simulator.haptic_aim_right:
                    meta_quest3.start_vibration(duration=0.1)
                    vibration_aim_detect = True
                    vibration_detect = True
            
            else:
                meta_quest3.stop_vibration()
                vibration_detect = False 
            

            if simulator.haptic_on_left :
                hand_left = input_data["left"]
                
                if hand_left["index_trigger"] and not vibration_detect_left:
                    meta_quest3.start_vibration(hand="left", duration=0.15)
                    vibration_detect_left = True
                    
                elif not hand_left["index_trigger"] :
                    meta_quest3.start_vibration(hand="left", duration=0.1)
                    vibration_detect_left = True
                    
                elif hand_left["index_trigger"] and simulator.haptic_aim_left:
                    meta_quest3.start_vibration(hand="left", duration=0.1)
                    vibration_aim_detect = True
                    vibration_detect_left = True
            
            else:
                meta_quest3.stop_vibration(hand="left")
                vibration_detect_left = False 
            
            simulator.next_step()
            recorder.record()
    elif args.i == 'real_robot':
    #    #real robot controller
        simulator: SFSimulator = sf_task_factory(args.t , host_address=args.host)
        recorder = SFRecorder(simulator)
        meta_quest3 = MetaQuest3("ALR2")
    #    #record,reset
        meta_quest3.register_button_press_event("X", recorder.save_record)
        meta_quest3.register_button_press_event(
            "X", simulator.reset_real_robot_in_the_main_thread
        )
        meta_quest3.register_trigger_press_event(
            "hand_trigger", "right", recorder.start_record
        )
        meta_quest3.register_trigger_release_event(
            "hand_trigger", "right", recorder.stop_record
        )
        robot_list: List[RobotBase] = list()
        real_robot_list: List[poly_controllers.Panda] = list()
        controller_list: List[ControllerBase] = list()
        real_robot_config = yaml.safe_load(
                open(
                    "./hdar_simulator/_simulationframework/task/" +
                    f"{args.t}/controller.yaml", "r"
                )
            )
        
        
        for robot_name, robot_config in simulator.robot_dict.items():
            real_robot = poly_controllers.Panda(
                name=robot_name,
                **real_robot_config[robot_name],
            )
            real_robot_list.append(real_robot)
            real_controller = controllers.RealRobotController(real_robot)
            vt_robot=robot_name
            vt_scene=simulator.mj_scene
            vt_controller = controllers.VTController(
                        vt_robot,
                        real_robot,
                        vt_scene,
                        robot_config,
                    )
            robot_list.append(vt_robot)
            controller_list.append(vt_controller)

        controller_dict = {vt_robot: controller 
                        for vt_robot, controller in zip(robot_list, controller_list)}
        simulator.assign_controller(controller_dict)

        force_interval = 0.02
        force_last_timestep = -force_interval

        for real_robot in real_robot_list:
            if real_robot.is_running_policy():
                real_robot.robot.terminate_current_policy()
        
        simulator.reset_real_robot()
        print(f"real robot go home pos {real_robot.robot.get_ee_pose()}")


        while True:
            if (
                vt_scene.time_stamp
                > force_last_timestep + force_interval
            ):
                force_last_timestep = vt_scene.time_stamp      
            
               
                for vt_robot, real_robot in zip(robot_list, real_robot_list):

                    real_robot1, real_robot2 = real_robot_list
                   
               
                    if real_robot.is_running_policy():
                        
                        for robot_name,robot_config in simulator.robot_dict.items():
                            if robot_name == "panda_robot1":
                                robot_config1 = robot_config
                            else :
                                robot_config2 = robot_config
                        
                                            

        #   # robot1
                        if vt_robot == "panda_robot1":
                            
                            constraint_forces = [
                                    vt_scene.data.joint(name).qfrc_constraint[0] + vt_scene.data.joint(name).qfrc_bias[0] * 0.25
                                    for name in robot_config1.joint_names
                                ]
                            output=[
                                    vt_scene.data.joint(name).qfrc_constraint[0] 
                                    for name in robot_config1.joint_names
                                ]
                            
        #   # withour object forcefeedback
                            # constraint_forces = [
                            #         vt_scene.data.joint(name).qfrc_bias[0] * 0.25
                            #         for name in robot_config1.joint_names
                            #     ]
                            constraint_forces = torch.tensor(np.array(constraint_forces))
                            try:
                                real_robot1.update_constraint_forces(constraint_forces)
                               
                            except:
                                pass
        #   # robot2
                        elif vt_robot == "panda_robot2":
                            constraint_forces = [
                                    vt_scene.data.joint(name).qfrc_constraint[0] + vt_scene.data.joint(name).qfrc_bias[0] * 0.25
                                    for name in robot_config2.joint_names
                                ]
                            
        #   # withour object forcefeedback
                            # constraint_forces = [
                            #         vt_scene.data.joint(name).qfrc_bias[0] * 0.25
                            #         for name in robot_config2.joint_names
                            #     ]
                            constraint_forces = torch.tensor(np.array(constraint_forces))
                            try:
                                real_robot2.update_constraint_forces(constraint_forces)
                                
                            except:
                                pass
            simulator.next_step()
            recorder.record()

        
    else:
        raise NotImplementedError("Only MetaQuest3 is supported for now.")

