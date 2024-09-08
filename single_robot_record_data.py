import argparse
from hdar_simulator.sf import sf_task_factory
from hdar_simulator.sf import SFRecorder
from hdar_simulator.sf import MetaQuest3Controller

from simpub.xr_device.meta_quest3 import MetaQuest3

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", default='BoxPushingSimulator'
    )
    parser.add_argument(
        "--host", default='127.0.0.1'
    )
    parser.add_argument("-i", default='meta_quest3')
    args = parser.parse_args()

    simulator = sf_task_factory(args.t, host_address=args.host)
    recorder = SFRecorder(simulator)

    if args.i == 'meta_quest3':
        meta_quest3 = MetaQuest3("ALRMetaQuest3")
        meta_quest3.register_button_press_event("X", recorder.save_record)
        meta_quest3.register_button_press_event(
            "X", simulator.reset_in_the_main_thread
        )
        meta_quest3.register_trigger_press_event(
            "hand", "right", recorder.start_record
        )
        meta_quest3.register_trigger_release_event(
            "hand", "right", recorder.stop_record
        )
        meta_controller = MetaQuest3Controller(
            meta_quest3,
            fix_rotation=True,
            with_hand=False,
        )
        simulator.assign_controller({"panda_robot": meta_controller})
    else:
        raise NotImplementedError("Only MetaQuest3 is supported for now.")

    simulator.run()
