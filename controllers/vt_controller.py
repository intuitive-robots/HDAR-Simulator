from alr_sim import controllers
import numpy as np
import time

from alr_sim.core import Scene, RobotBase
from alr_sim.controllers.IKControllers import CartPosQuatImpedenceController
import threading

#use to set the 
class VTController(controllers.JointPDController):
    def __init__(
        self,
        vt_robot,
        real_robot,
        vt_scene: Scene,
        robot_config: dict,
        use_gripper: bool = True,
        update_interval: int = 10,
    ):
        super().__init__()
        self.vt_robot = vt_robot
        self.real_robot = real_robot
        self.robot_config = robot_config
        self.use_gripper = use_gripper
        self.pgain = np.array(
            [2000.0, 2000.0, 2000.0, 2000.0, 200.0, 100.0, 30.0]
        )  # np.array([120.0, 120.0, 120.0, 120.0, 50.0, 30.0, 10.0])
        self.dgain = np.array([10.0, 10.0, 10.0, 10.0, 6.0, 5.0, 3.0]) / 2

        self.t = 0
        self.update_interval = update_interval

        # self.viewController = MjViewGamePadController(vt_scene)

    def getControl(self, robot: RobotBase):
        # self.viewController.controlViewCamera()
        if self.t % self.update_interval == 0:
            self.setSetPoint(
                desired_pos=self.real_robot.robot.get_joint_positions().numpy(),
                desired_vel=self.real_robot.robot.get_joint_velocities().numpy(),
            )
            
            if self.use_gripper:
                if self.real_robot.gripper.get_state().width <= 0.075:
                    robot.set_desired_gripper_width(0)
                else:
                    robot.open_fingers()
        self.t += 1
        return super().getControl(robot)

    # def reset_robot(self):
    #     if self.real_robot.is_running_policy():
    #         self.real_robot.robot.terminate_current_policy()
    #     self.real_robot.robot.go_home(blocking=False)
    #     threading.Thread(target=self._reset_robot).start()

    # def _reset_robot(self):
    #     time.sleep(0.2)
    #     i = 0
    #     while not self.real_robot.is_running_policy() and i < 20:
    #         self.real_robot.robot.go_home(blocking=False)
    #         time.sleep(0.2)
    #         i += 1

    #     while self.real_robot.is_running_policy():
    #         time.sleep(0.1)

    #     self.real_robot.load_policy(blocking=False)
    #     i = 0
    #     while not self.real_robot.is_running_policy() and i < 20:
    #         self.real_robot.load_policy(blocking=False)
    #         time.sleep(0.2)
    #         i += 1

#TODO reset the real-robot pos as in the vt_robot_reset 
    def reset_robot(self,target_pos=None,target_orientation=[0, 1, 0, 0]):
    # def reset_robot(self,target_pos=None,target_orientation=[1, 0, 0, 0]):
        if self.real_robot.is_running_policy():
            # self.real_robot.robot.terminate_current_policy()
            self.real_robot.robot.go_home(blocking=False)
            
        if target_pos is not None:
            #TODO find how to move?
            self.real_robot.robot.go_home(blocking=False)
            
            # self.real_robot.robot.move_to_ee_pose(position=target_pos, orientation=target_orientation,time_to_go=2.0)
        else:
            self.real_robot.robot.go_home(blocking=False)
            
        threading.Thread(target=self._reset_robot(target_pos==target_pos, target_orientation=target_orientation)).start()

    def _reset_robot(self,target_pos=None,target_orientation=[0, 1, 0, 0]):
        time.sleep(0.2)
        i = 0
        while not self.real_robot.is_running_policy() and i < 5:
            # if target_pos is not None:
            #     #TODO find how to move?move to ee pos?need input :ee_position ,ee_quat
            #     self.real_robot.robot.go_home(blocking=False)
            #     print(f"real robot go home pos {self.real_robot.robot.get_ee_pose}")
            #     # self.real_robot.robot.move_to_ee_pose(position=target_pos, orientation=target_orientation)
            #     time.sleep(0.2)
            #     i += 1
            # else:
                
                self.real_robot.robot.go_home(blocking=False)
                
                print(f"real robot go home pos {self.real_robot.robot.get_ee_pose}")
                time.sleep(0.2)
                i += 1

        while self.real_robot.is_running_policy():
            time.sleep(0.1)
            # self.real_robot.robot.terminate_current_policy()
            # self.real_robot.robot.go_home(blocking=False)
        

        self.real_robot.load_policy(blocking=False)
        # self.real_robot.robot.go_home(blocking=False)
        time.sleep(0.2)
        i = 0
        while not self.real_robot.is_running_policy() and i < 5:
            self.real_robot.load_policy(blocking=False)
            time.sleep(0.2)
            i += 1


class VTController_fixtcpquat(CartPosQuatImpedenceController):
    def __init__(
        self,
        vt_robot,
        real_robot,
        vt_scene: Scene,
        robot_config: dict,
        use_gripper: bool = True,
        update_interval: int = 10,
    ):
        super().__init__()
        # self.vt_robot = vt_robot
        self.real_robot = real_robot
        self.robot_config = robot_config
        self.use_gripper = use_gripper
        self.pgain = np.array(
            [2000.0, 2000.0, 2000.0, 2000.0, 200.0, 100.0, 30.0]
        )  # np.array([120.0, 120.0, 120.0, 120.0, 50.0, 30.0, 10.0])
        self.dgain = np.array([10.0, 10.0, 10.0, 10.0, 6.0, 5.0, 3.0]) / 2

        self.t = 0
        self.update_interval = update_interval

        # self.viewController = MjViewGamePadController(vt_scene)

    def getControl(self, robot: RobotBase):
        # self.viewController.controlViewCamera()
        if self.t % self.update_interval == 0:
            # self.setSetPoint(
            #     desired_pos=self.real_robot.robot.get_joint_positions().numpy(),
            #     desired_vel=self.real_robot.robot.get_joint_velocities().numpy(),
            # )
            real_robot_pos,real_robot_quat = self.real_robot.robot.get_ee_pose()
            desired_pos = [real_robot_pos[0],real_robot_pos[1],0.28]
            desired_quat = [0,1,0,0]
            self.setSetPoint(np.hstack((desired_pos, desired_quat)))
            if self.use_gripper:
                if self.real_robot.gripper.get_state().width <= 0.075:
                    robot.set_desired_gripper_width(0)
                else:
                    robot.open_fingers()
        self.t += 1
        return super().getControl(robot)
    
    def reset_robot(self,target_pos=None,target_orientation=[0, 1, 0, 0]):
    
        if self.real_robot.is_running_policy():
            self.real_robot.robot.terminate_current_policy()
        if target_pos is not None:
            #TODO find how to move?
            # self.real_robot.robot.go_home(blocking=False)
            self.real_robot.robot.move_to_ee_pose(position=[0.53,-0.09,0.28],orientation=target_orientation)
            # self.real_robot.robot.move_to_ee_pose(position=target_pos, orientation=target_orientation,time_to_go=2.0)
        else:
            self.real_robot.robot.move_to_ee_pose(position=[0.53,-0.09,0.28],orientation=target_orientation)
            
        threading.Thread(target=self._reset_robot(target_pos==target_pos, target_orientation=target_orientation)).start()

    def _reset_robot(self,target_pos=None,target_orientation=[0, 1, 0, 0]):
        time.sleep(0.2)
        i = 0
        while not self.real_robot.is_running_policy() and i < 0:
            # if target_pos is not None:
            #     #TODO find how to move?move to ee pos?need input :ee_position ,ee_quat
            #     self.real_robot.robot.go_home(blocking=False)
            #     print(f"real robot go home pos {self.real_robot.robot.get_ee_pose}")
            #     # self.real_robot.robot.move_to_ee_pose(position=target_pos, orientation=target_orientation)
            #     time.sleep(0.2)
            #     i += 1
            # else:
                
                # self.real_robot.robot.go_home(blocking=False)
                self.real_robot.robot.move_to_ee_pose(position=[0.53,-0.09,0.28],orientation=target_orientation)
                print(f"real robot go home pos {self.real_robot.robot.get_ee_pose}")
                time.sleep(0.2)
                i += 1

        while self.real_robot.is_running_policy():
            time.sleep(0.1)
            # self.real_robot.robot.terminate_current_policy()
            # self.real_robot.robot.go_home(blocking=False)
        

        self.real_robot.load_policy(blocking=False)
        # self.real_robot.robot.go_home(blocking=False)
        time.sleep(0.2)
        i = 0
        while not self.real_robot.is_running_policy() and i < 5:
            self.real_robot.load_policy(blocking=False)
            time.sleep(0.2)
            i += 1