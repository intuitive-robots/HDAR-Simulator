# import utils, simulator


# if __name__ == "__main__":
#     simulator_config = utils.get_simulator_config()
#     simulator.Simulation(**simulator_config).run()

import argparse
import os

from hdar_simulator.sf.task import sf_task_factory
from hdar_simulator.sf.sf_replayer import SFReplayer

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", default='BoxPushingSimulator'
    )
    parser.add_argument(
        "-s", default='SF'
    )
    # parser.add_argument("-i",)
    args = parser.parse_args()

    if args.s == 'SF':
        hdar_path = os.path.dirname(os.path.abspath(__file__))
        data_path = os.path(hdar_path, "/SFDemoData/")
        replayer = SFReplayer()
    else:
        raise NotImplementedError("Only SF simulator is supported for now.")

    simulator = replayer.create_simulator(sf_task_factory(args.t))
    replayer.replay()
