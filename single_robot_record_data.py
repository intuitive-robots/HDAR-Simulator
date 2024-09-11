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

    simulator: SFSimulator = sf_task_factory(args.t, host_address=args.host)
    recorder = SFRecorder(simulator)
    controller_config = yaml.safe_load(
        open(
            "./hdar_simulator/_simulationframework/task/" +
            f"{args.t}/controller.yaml", "r"
        )
    )
    if args.i == 'meta_quest3':
        meta_quest3 = MetaQuest3("ALR2")
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
        meta_controller = MetaQuest3Controller(
            meta_quest3,
            **controller_config["meta_quest3"],
        )
        simulator.assign_controller({"panda_robot": meta_controller})
    else:
        raise NotImplementedError("Only MetaQuest3 is supported for now.")

    simulator.reset()
    while True:
        simulator.next_step()
        recorder.record()
