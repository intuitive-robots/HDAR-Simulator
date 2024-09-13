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
        "-t", default='BoxPushing'
    )
    parser.add_argument(
        "-s", default='SF'
    )
    # parser.add_argument("-i",)
    args = parser.parse_args()

    if args.s == 'SF':
        hdar_path = os.path.dirname(os.path.abspath(__file__))
        print(hdar_path)
        data_path = os.path.join(hdar_path, "SFDemoData/", "PutIteminDrawer_2024_09_12_17_31_51/PutIteminDrawer_001.pkl")
        print(data_path)
        replayer = SFReplayer()
    else:
        raise NotImplementedError("Only SF simulator is supported for now.")

    simulator = replayer.create_simulator(data_path)
    replayer.replay()
