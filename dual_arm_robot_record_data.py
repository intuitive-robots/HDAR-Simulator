import argparse
import yaml

from hdar_simulator._simulationframework import sf_task_factory
from hdar_simulator._simulationframework import SFRecorder
from hdar_simulator._simulationframework import MetaQuest3Controller
from hdar_simulator._simulationframework import SFSimulator
from simpub.xr_device.meta_quest3 import MetaQuest3

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", default='BoxPushing'
    )
    parser.add_argument(
        "--host", default='127.0.0.1'
    )
    parser.add_argument("-i", default='meta_quest3')
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
        # simulator.assign_controller({"panda_robot2": meta_controller2})
    elif args.i == 'real_robot':
        #TODO real robot controller
        pass
    else:
        raise NotImplementedError("Only MetaQuest3 is supported for now.")
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
