# import utils, simulator


# if __name__ == "__main__":
#     simulator_config = utils.get_simulator_config()
#     simulator.Simulation(**simulator_config).run()

import argparse
from simulator.sim.sf.task import sf_task_factory

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
# #Test Task:
        # "-t", choices=['BoxPushingSimulator'], default='BoxPushingSimulator'

# #Single-Arm Task:
        # "-t", choices=['OpenDrawerSimulator'], default='OpenDrawerSimulator'
        # "-t", choices=['OpenDoorSimulator'], default='OpenDoorSimulator'
        # "-t", choices=['BoxPushSimulator'], default='BoxPushSimulator'
        # "-t", choices=['BoxAssembleSimulator'], default='BoxAssembleSimulator'
        # "-t", choices=['BoxPickandPlaceSimulator'], default='BoxPickandPlaceSimulator'
        
# #Dual-Arm Task:    
        "-t", choices=['BimanualAssembleSimulator'], default='BimanualAssembleSimulator'
        # "-t", choices=['BimanualPushingSimulator'], default='BimanualPushingSimulator'
        # "-t", choices=['BimanualHoldingSimulator'], default='BimanualHoldingSimulator'
        # "-t", choices=['BimanualPutIteminDrawSimulator'], default='BimanualPutIteminDrawSimulator'
    
    )
    # parser.add_argument("-i",)
    args = parser.parse_args()

    simulator = sf_task_factory(args.t)
    simulator.run()
