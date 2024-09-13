import argparse
import yaml

from hdar_simulator._simulationframework import sf_task_factory
from hdar_simulator._simulationframework import SFRecorder
from hdar_simulator._simulationframework import MetaQuest3Controller
from hdar_simulator._simulationframework import SFSimulator
from simpub.xr_device.meta_quest3 import MetaQuest3
import poly_controllers,controllers

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", default='BoxPushing'
    )
    parser.add_argument(
        "--host", default='127.0.0.1'
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
        simulator: SFSimulator = sf_task_factory(modified_task_type, host_address=args.host)
        recorder = SFRecorder(simulator)
        controller_config = yaml.safe_load(
            open(
                "./hdar_simulator/_simulationframework/task/" +
                f"{args.t}/controller.yaml", "r"
            )
        )
        meta_quest3 = MetaQuest3("ALR1")
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
        simulator.assign_controller({"panda_robot": meta_controller})

        vibration_detect = False
        vibration_aim_detect = False
        simulator.reset()
        while True:
            simulator.after_step()
            input_data=meta_quest3.get_input_data()
            if simulator.haptic_on :
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
        #TODO real robot controller
        simulator: SFSimulator = sf_task_factory(args.t , host_address=args.host)
        recorder = SFRecorder(simulator)
        for robot_name, robot_config in simulator.create_robots():
            vt_robot=simulator.create_robots[robot_name]
            real_robot_config = yaml.safe_load(
                open(
                    "./hdar_simulator/_simulationframework/task/" +
                    f"{args.t}/controller.yaml", "r"
                )
            )
            real_robot = poly_controllers.Panda(
                name=robot_name,
                **real_robot_config,
            )
            real_controller = controllers.RealRobotController(real_robot)
            # vt_robot=simulator.create_robots()
            vt_scene=simulator.mj_scene
            vt_controller = controllers.VTController(
                        # vt_robot,
                        real_robot,
                        vt_scene,
                        robot_config,
                    )

            simulator.assign_controller({"panda_robot": vt_controller})

        simulator.reset()
        while True:
            simulator.next_step()
            recorder.record()
    else:
        raise NotImplementedError("Only MetaQuest3 is supported for now.")
