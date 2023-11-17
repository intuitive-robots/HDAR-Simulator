
from alr_sim.controllers import JointPDController
from alr_sim.sims.sl.multibot_teleop.src.human_controller import HumanController
from alr_sim.sims.sl import SlRobot

from interface_base import *
from ..server import HDARFactory


class VTController(JointPDController):
    def __init__(
        self,
        sl_robot: SlRobot,
    ):
        super().__init__()
        self.sl_robot = sl_robot
        self.goal_pos_fct = lambda: sl_robot.current_j_pos
        self.goal_vel_fct = lambda: sl_robot.current_j_vel
        self.goal_gripper_width_fct = lambda: sl_robot.gripper_width

    def getControl(self, robot: MjRobot):
        self.setSetPoint(
            desired_pos=self.goal_pos_fct(), desired_vel=self.goal_vel_fct()
        )
        if self.goal_gripper_width_fct() <= 0.075:
            robot.close_fingers(duration=0)  # Grasp
        else:
            robot.open_fingers()
        return super().getControl(robot)

    # def reset_robot(self):
        # pos_local, quat_local = self.sl_robot._localize_cart_coords(
        #     self.robot_config["init_end_eff_pos"],
        #     self.robot_config["init_end_eff_quat"],
        # )
        # self.sl_robot.gotoCartPositionAndQuat(
        #     pos_local, quat_local, duration=3, global_coord=True, block=True
        # )
        # self.sl_robot.activeController = self


class KinestheticTeachingInterface(InterfaceBase):
    
    def __init__(self, mj_robot, sl_config) -> None:
        super().__init__(mj_robot)
        self.sl_scene = HDARFactory.create_sl_scene()
        self.sl_robot = HDARFactory.create_sl_robot(mj_robot, sl_config)
        self.sl_robot_controller = HumanController(
            self.sl_robot,
            regularize=True,
        )
        self.mj_robot_controller = VTController(self.sl_robot)


    def initialise_controller(self) -> None:
        self.sl_robot_controller.executeController(
            self.mj_robot, maxDuration=1000, block=False
        )
        super().initialise_controller()

