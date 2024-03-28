from alr_sim import controllers
import numpy as np
import time

from alr_sim.core import Scene, RobotBase


class VTController(controllers.JointPDController):
    def __init__(
        self,
        scene: Scene,
        robot,
        robot_config: dict,
        use_gripper: bool = True,
        update_interval: int = 10,
    ):
        super().__init__()
        self.robot = robot
        self.robot_config = robot_config
        self.use_gripper = use_gripper
        self.pgain = np.array(
            [2000.0, 2000.0, 2000.0, 2000.0, 200.0, 100.0, 30.0]
        )  # np.array([120.0, 120.0, 120.0, 120.0, 50.0, 30.0, 10.0])
        self.dgain = np.array([10.0, 10.0, 10.0, 10.0, 6.0, 5.0, 3.0]) / 2

        self.real_joint_pos = robot.get_init_qpos()
        self.real_joint_vel = np.zeros(7)
        self.real_gripper_width = 0

        self.reset_flag = False
        self.first = True

        # self.viewController = MjViewGamePadController(vt_scene)
    
    def update_real_joints(self, joint_info):
        self.real_joint_pos = joint_info['joint_pos']
        self.real_joint_vel = joint_info['joint_vel']
        self.real_gripper_width = joint_info['gripper_width']

        if self.first:
            self.first = False
            self.robot.beam_to_joint_pos(joint_info['joint_pos'], run=False)
            self.desired_joint_pos = np.array(self.real_joint_pos)
            self.desired_joint_vel = np.zeros(7)
            self.robot.current_j_pos = np.array(self.real_joint_pos)
            self.robot.current_j_vel = np.array(self.real_joint_vel)
            time.sleep(0.1)
            self.executeController(self.robot, 1000, block=False)
    
    def getControl(self, robot: RobotBase):
        # self.viewController.controlViewCamera()
        self.setSetPoint(desired_pos=self.real_joint_pos, desired_vel=self.real_joint_vel)
        if self.use_gripper:
            if self.real_gripper_width <= 0.075:
                robot.set_desired_gripper_width(0)
            else:
                robot.open_fingers()
        return super().getControl(robot)

    def reset_robot(self):
        self.reset_flag = True
        