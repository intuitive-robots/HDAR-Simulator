import torch
import spdlog
import signal
import threading

import torchcontrol as toco

from poly_controllers.panda import Panda
from poly_controllers.controllers import HumanController, ImitationController
from polymetis import RobotInterface


GRIPPER_LOWER_BOUND = 0.01
GRIPPER_UPPER_BOUND = 0.075
GRIPPER_SPEED = 1.0
GRIPPER_FORCE = 20.0
GRIPPER_GRASP_THRESHOLD = 0.04
GRIPPER_OPEN_THRESHOLD = 0.05


class Teleoperation:
    def __init__(self, operated_panda: Panda, imitator_panda: Panda):
        self.logger = spdlog.ConsoleLogger("teleoperation")
        self.logger.set_level(spdlog.LogLevel.DEBUG)
        self.operated_panda = operated_panda
        self.imitator_panda = imitator_panda

        self.human_controller = HumanController(self.operated_panda.robot)
        self.imitation_controller = ImitationController(self.imitator_panda.robot)

        # Reset imitator to operated robots current positions
        self.logger.info(
            f"Moving {self.imitator_panda.name} to {self.operated_panda.name}'s joint positions"
        )
        self.imitator_panda.robot.move_to_joint_positions(
            self.operated_panda.robot.get_joint_positions()
        )

        self.logger.info("Sending torch policies")
        self.operated_panda.load_policy(self.human_controller, blocking=False)
        self.imitator_panda.load_policy(self.imitation_controller, blocking=False)

        self.running = True

        # setup up signal for cleanup
        signal.signal(signal.SIGINT, self.__sig_handler)

    def run(self):
        self.running = True
        last_command = "move"
        last_issued = -1
        while self.running:
            if not self.operated_panda.is_running_policy():
                self.operated_panda.load_policy()

            if not self.imitator_panda.is_running_policy():
                self.imitator_panda.load_policy()

            # update robot target position
            joint_pos_desired = self.operated_panda.robot.get_joint_positions()
            self.imitator_panda.robot.update_desired_joint_positions(joint_pos_desired)

            # mirror gripper state for now only binary (grasping fully or moving to fully open)
            desired_gripper_width = self.operated_panda.get_state()["gripper"].width

            if (
                desired_gripper_width < GRIPPER_GRASP_THRESHOLD
                and last_command == "move"
            ):
                self.logger.debug("Grasping")
                self.imitator_panda.gripper.grasp(
                    grasp_width=GRIPPER_LOWER_BOUND,
                    speed=GRIPPER_SPEED,
                    force=GRIPPER_FORCE,
                    epsilon_inner=0.01,
                    epsilon_outer=0.2,
                )

                last_command = "grasp"

            elif (
                desired_gripper_width > GRIPPER_OPEN_THRESHOLD
                and last_command == "grasp"
            ):
                self.logger.debug("Moving")
                self.imitator_panda.gripper.goto(
                    width=GRIPPER_UPPER_BOUND,
                    speed=GRIPPER_SPEED,
                    force=GRIPPER_FORCE,
                )

                last_command = "move"

    def __sig_handler(self, _sig, _frame):
        self.running = False

    def cleanup(self):
        self.logger.info("Terminating policies")
        self.operated_panda.robot.terminate_current_policy()
        self.imitator_panda.robot.terminate_current_policy()
