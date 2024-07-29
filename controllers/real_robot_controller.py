import poly_controllers
import time
import threading

#physical joint forcefeedback 
class RealRobotController(poly_controllers.ForceFeedbackController):
    def __init__(self, robot, regularize=True):
        super().__init__(robot.robot, regularize)
        self.robot = robot
        self.start()
    
    def start(self):
        threading.Thread(target=self._start).start()
    
    def _start(self):
        self.robot.load_policy(self, blocking=False)
        time.sleep(0.3)
        i = 0
        while not self.robot.is_running_policy() and i < 20:
            self.robot.load_policy(self, blocking=False)
            time.sleep(0.3)
            i += 1
