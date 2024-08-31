# import utils, simulator


# if __name__ == "__main__":
#     simulator_config = utils.get_simulator_config()
#     simulator.Simulation(**simulator_config).run()

from simulator.sim.SimulationFramework import BoxPushingSimulator
from simulator.sim.SimulationFramework import BoxPickandPlaceSimulator

if __name__ == '__main__':
    # simulator = BoxPushingSimulator()
    simulator = BoxPickandPlaceSimulator()
    simulator.run()
