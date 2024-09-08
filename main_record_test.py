# import utils, simulator


# if __name__ == "__main__":
#     simulator_config = utils.get_simulator_config()
#     simulator.Simulation(**simulator_config).run()

import argparse
from hdar_simulator.sim.sf.task import sf_task_factory

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", choices=['BoxPushing'], default='BoxPushing'
    )
    # parser.add_argument("-i",)
    args = parser.parse_args()

    simulator = sf_task_factory(args.t)
    simulator.recorder.start_record()
    for _ in range(1000):
        simulator.before_step()
        simulator.mj_scene.next_step()
        if simulator.record_mode:
            simulator.recorder.record()
        simulator.after_step()
    simulator.recorder.save_record()
