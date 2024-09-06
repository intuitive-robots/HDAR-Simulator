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
        # "-t", default='BoxPushingSimulator'

        # "-t", default='OpenDrawerSimulator'
        "-t", default='OpenDoorSimulator'
        # "-t", default='BoxPushSimulator'
        # "-t", default='BoxAssembleSimulator'
        # "-t", default='BoxPickandPlaceSimulator'

        # "-t", default='BimanualAssembleSimulator'
        # "-t", default='BimanualPushingSimulator'
        # "-t", default='BimanualHoldingSimulator'
        # "-t", default='BimanualPutIteminDrawSimulator'

    )
    # parser.add_argument("-i",)
    args = parser.parse_args()

    replayer = SFReplayer()
    replayer.load_data("/home/xueyinli/project/HDAR-Simulator/SFDemoData/OpenDoor_2024_09_06_15_34_48/OpenDoor_000.pkl")
    replayer.replay_data()
