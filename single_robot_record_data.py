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


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", default='PushCube'
    )
    parser.add_argument(
        "--host", default='192.168.0.143'
    )
    # parser.add_argument("-i", default='meta_quest3')
    parser.add_argument("-i", default='real_robot')
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
        # modified_task_type = args.t 
        simulator: SFSimulator = sf_task_factory(modified_task_type, host_address=args.host)
        recorder = SFRecorder(simulator)
        controller_config = yaml.safe_load(
            open(
                "./hdar_simulator/_simulationframework/task/" +
                f"{args.t}/controller.yaml", "r"
            )
        )
        meta_quest3 = MetaQuest3("UnityClient")
        #record,reset
        meta_quest3.register_button_press_event("X", recorder.save_record)
        meta_quest3.register_button_press_event(
            "X", simulator.reset_in_the_main_thread
        )
        meta_quest3.register_trigger_press_event(
            "hand_trigger", "right", recorder.start_record
        )
        meta_quest3.register_trigger_release_event(
            "hand_trigger", "right", recorder.stop_record
        )
        #Set controller
        meta_controller = MetaQuest3Controller(
            meta_quest3,
            **controller_config["meta_quest3"],
        )
        meta_robot = "panda_robot"
        simulator.assign_controller({ meta_robot : meta_controller } )
      

        vibration_detect = False
        vibration_aim_detect = False
        simulator.reset()
        while True:
            simulator.after_step()
            
            if simulator.haptic_on :
                input_data=meta_quest3.get_input_data()
                hand_right = input_data["right"]
                
                if hand_right["index_trigger"] and not vibration_detect:
                    meta_quest3.start_vibration(duration=0.15)
                    vibration_detect = True
                    
                elif not hand_right["index_trigger"] :
                    meta_quest3.start_vibration(duration=0.1)
                    vibration_detect = True
                    
                elif hand_right["index_trigger"] and simulator.haptic_aim:
                    meta_quest3.start_vibration(duration=0.1)
                    vibration_aim_detect = True
                    vibration_detect = True
            
            else:
                meta_quest3.stop_vibration()
                vibration_detect = False 
            
            simulator.next_step()
            recorder.record()
    elif args.i == 'real_robot':
        #real robot controller
        simulator: SFSimulator = sf_task_factory(args.t , host_address=args.host)
        recorder = SFRecorder(simulator)
        meta_quest3 = MetaQuest3("UnityClient")
        #record,reset
        meta_quest3.register_button_press_event("X", recorder.save_record)
        meta_quest3.register_button_press_event(
            "X", simulator.reset_real_robot_in_the_main_thread
        )
    

        # Record while the 'R' key is held down
        
        meta_quest3.register_trigger_press_event(
            "hand_trigger", "right", recorder.start_record
        )
        meta_quest3.register_trigger_release_event(
            "hand_trigger", "right", recorder.stop_record
        )
        for robot_name, robot_config in simulator.robot_dict.items():
            real_robot_config = yaml.safe_load(
                open(
                    "./hdar_simulator/_simulationframework/task/" +
                    f"{args.t}/controller.yaml", "r"
                )
            )
            real_robot = poly_controllers.Panda(
                name=robot_name,
                **real_robot_config["real_robot"],
            )
            real_controller = controllers.RealRobotController(real_robot)
            vt_robot=robot_name
            vt_scene=simulator.mj_scene
            vt_controller = controllers.VTController(
                        vt_robot,
                        real_robot,
                        vt_scene,
                        robot_config,
                    )
        simulator.assign_controller({vt_robot: vt_controller})

        force_interval = 0.02
        force_last_timestep = -force_interval
        if real_robot.is_running_policy():
            real_robot.robot.terminate_current_policy()

    #    # reset real_robot
        simulator.reset_real_robot()
        print(f"real robot go home pos {real_robot.robot.get_ee_pose()}")



        while True:
            # simulator.next_step()
            if (
                vt_scene.time_stamp
                > force_last_timestep + force_interval
            ):
                force_last_timestep = vt_scene.time_stamp
                if real_robot.is_running_policy():
                    constraint_forces = [
                            vt_scene.data.joint(name).qfrc_constraint[0] *1.5 + vt_scene.data.joint(name).qfrc_bias[0] * 0.25
                            for name in robot_config.joint_names
                        ]
  # withour object forcefeedback
                    # constraint_forces = [
                    #         vt_scene.data.joint(name).qfrc_bias[0] * 0.25
                    #         for name in robot_config.joint_names
                    #     ]
                    constraint_forces = torch.tensor(np.array(constraint_forces))
                    try:
                        real_robot.update_constraint_forces(constraint_forces)
                    except:
                        pass
            simulator.next_step()
            recorder.record()
    else:
        raise NotImplementedError("Only MetaQuest3 is supported for now.")
