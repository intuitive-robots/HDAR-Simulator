import abc

from alr_sim.sims.mj_beta import MjRobot
from alr_sim.controllers import ControllerBase

class InterfaceBase(abc.ABC):
    
    def __init__(self, mj_robot: MjRobot) -> None:
        super().__init__()
        self.mj_robot: MjRobot = mj_robot
        self.mj_robot_controller: ControllerBase

    def initialise_controller(self) -> None:
        self.mj_robot_controller.executeController(
            self.mj_robot, maxDuration=1000, block=False
        )