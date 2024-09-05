# import utils, simulator


# if __name__ == "__main__":
#     simulator_config = utils.get_simulator_config()
#     simulator.Simulation(**simulator_config).run()

import argparse
from simulator.sim.sf.task import sf_task_factory
from simulator.sim.sf.SFReplayer import SFReplayer

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", choices=['BoxPushingSimulator'], default='BoxPushingSimulator'
    )
    # parser.add_argument("-i",)
    args = parser.parse_args()

    replayer = SFReplayer()
    replayer.load_data("/home/xinkai/project/HDAR-Simulator/SFDemoData/BoxPushing_2024_09_05_12_49_34/BoxPushing_000.pkl")
    replayer.replay_data()
