import utils, simulation


if __name__ == "__main__":
    simulator_config = utils.get_simulator_config()
    simulation.Simulation(**simulator_config).run()
  
  