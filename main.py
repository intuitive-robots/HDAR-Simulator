# import utils, simulator


# if __name__ == "__main__":
#     simulator_config = utils.get_simulator_config()
#     simulator.Simulation(**simulator_config).run()

from simulator.env.SimulationFramework import BoxPushingSimulator

if __name__ == '__main__':
    simulator = BoxPushingSimulator()
    simulator.run()
    