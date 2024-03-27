import poly_controllers
import time
import threading


def start_poly_controller(robot, controller):
    def _start():
        robot.load_policy(controller, blocking=False)
        time.sleep(0.3)
        i = 0
        while not robot.is_running_policy() and i < 20:
            robot.load_policy(controller, blocking=False)
            time.sleep(0.3)
            i += 1

    threading.Thread(target=_start).start()


class RealRobotController(poly_controllers.controllers.ForceFeedbackController):
    def __init__(self, primary_robot, regularize=True):
        super().__init__(primary_robot, regularize)