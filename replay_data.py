# import utils, simulator


# if __name__ == "__main__":
#     simulator_config = utils.get_simulator_config()
#     simulator.Simulation(**simulator_config).run()

import argparse
import os

from hdar_simulator._simulationframework.task import sf_task_factory
from hdar_simulator._simulationframework.sf_replayer import SFReplayer

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", default='BoxPushingV2'
    )
    parser.add_argument(
        "-s", default='SF'
    )
    # parser.add_argument("-i",)
    args = parser.parse_args()
    task_name = args.t
    if args.s == 'SF':
        hdar_path = os.path.dirname(os.path.abspath(__file__))
        data_path = os.path.join(hdar_path, "SFDemoData/", task_name)
        replayer = SFReplayer()
    else:
        raise NotImplementedError("Only SF simulator is supported for now.")

    simulator = None
    for file_name in os.listdir(data_path):
        file_path = os.path.join(data_path, file_name)
        if simulator is None:
            simulator = replayer.create_simulator(file_path)
        else:
            replayer.reset_simulator_from_new_data(file_path)
        print(f"Replaying {file_name}")
        replayer.replay()
