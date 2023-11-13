import numpy as np
import abc
import mujoco
from math import sin, cos, radians, degrees

from alr_sim.core import Scene, RobotBase
from alr_sim.sims.mj_beta import MjScene
from alr_sim.sims.sl.SlRobot import SlRobot
from alr_sim.controllers.IKControllers import CartPosQuatImpedenceController
from alr_sim.controllers.Controller import JointPDController
from alr_sim.sims.sl.multibot_teleop.src.human_controller import HumanController
from alr_sim.sims.mujoco.mj_interactive.devices.gamepad import GamePad
from alr_sim.sims.universal_sim.PrimitiveObjects import Box, Sphere
from alr_sim.utils import (
    geometric_transformation as geo_trans,
    euler2quat,
    euler2mat,
    mat2quat,
    quat2mat,
    quat_mul,
)
from alr_sim.sims.sl.multibot_teleop.src.kalman_filter import KalmanFilter

from .UnityStreamer import UnityStreamer, RobotHandler
from .triad_openvr import triad_openvr
from .utils import degEuler2radEuler


class BinarySingal:
    def __init__(
        self, signal_fct, state_list: list = [False, True], init_state=None
    ) -> None:
        self.last_signal = signal_fct()
        self.current_signal = None
        self.signal_fct = signal_fct
        self.index = 0 if init_state is None else state_list.index(init_state)
        self.state_list = state_list

    def update(self) -> None:
        self.current_signal = self.signal_fct()
        if self.current_signal and not self.last_signal:
            self.index += 1
            self.index %= len(self.state_list)
        self.last_signal = self.current_signal

    def get_state(self):
        return self.state_list[self.index]

    def get_current_signal(self):
        return self.current_signal


class CartKalmanFilter(KalmanFilter):
    def __init__(self, X, dt=0.001, x_cov=9e-3, dx_cov=0, obs_noise_cov=0.0012):
        # Time interval
        self.sz = X.shape[0]

        # State vector
        self.X = np.append(X, np.zeros((self.sz,)))

        # Motion Model
        self.F = np.diag(
            np.ones(
                2 * self.sz,
            )
        )
        self.F[: self.sz, self.sz :] = np.diag(np.full((self.sz,), dt))

        # Motion Noise Covariance
        self.Q = np.diag(
            np.concatenate([np.full((self.sz,), x_cov), np.full((self.sz,), dx_cov)])
        )

        # Correlation Matrix
        self.P = self.Q

        # Observation Model
        self.H = np.zeros((self.sz, 2 * self.sz))
        np.fill_diagonal(self.H, 1)

        # Observation Noise Covariance (load - grav)
        self.R = np.diag(np.full((self.sz,), obs_noise_cov))

        self.S = np.zeros((self.sz, self.sz))
        self.K = self.X


class MjViewGamePadController:
    def __init__(self, scene: MjScene, device: GamePad = None) -> None:
        self.scene = scene
        self.device = device
        if device is None:
            self.device = GamePad()
        self.view_mode: bool = False
        self.on_view_mode_triggered: bool = False

        self.signal_list: list[BinarySingal] = []
        self.view_signal = BinarySingal(self.btn_b)
        self.signal_list.append(self.view_signal)

        self.change_view_signal = BinarySingal(
            self.btn_a,
            ["front", "top", "left"],
        )
        self.signal_list.append(self.change_view_signal)

    def view_button(self):
        return self.device.BTN_EAST == 1

    def btn_a(self):
        return self.device.BTN_SOUTH == 1

    def btn_b(self):
        return self.device.BTN_EAST == 1

    def btn_x(self):
        return self.device.BTN_NORTH == 1

    def btn_y(self):
        return self.device.BTN_WEST == 1

    def change2top_view(self):
        self.scene.viewer._init_camera()
        cam = self.scene.viewer.cam
        cam.azimuth = 180
        cam.elevation = -90

    def change2left_view(self):
        self.scene.viewer._init_camera()
        cam = self.scene.viewer.cam
        cam.azimuth = 90
        cam.elevation = -30

    def change2front_view(self):
        self.scene.viewer._init_camera()
        cam = self.scene.viewer.cam
        cam.azimuth = 180
        cam.elevation = -30

    def get_view_direction(self):
        cam = self.scene.viewer.cam
        return radians(cam.azimuth), radians(cam.elevation)

    def controlViewCamera(self) -> False:
        for signal in self.signal_list:
            signal.update()

        if self.change_view_signal.current_signal:
            if self.change_view_signal.get_state() == "top":
                self.change2top_view()
            elif self.change_view_signal.get_state() == "left":
                self.change2left_view()
            else:
                self.change2front_view()

        if not self.view_signal.get_state():
            return False

        zoom_signal = self.device.ABS_RZ - self.device.ABS_Z

        if abs(zoom_signal) > 0.1:
            mujoco.mjv_moveCamera(
                self.scene.model,
                mujoco.mjtMouse.mjMOUSE_ZOOM,
                0,
                0.0001 * (self.device.ABS_RZ - self.device.ABS_Z),
                self.scene.viewer.scn,
                self.scene.viewer.cam,
            )

        translate_x, translate_y = self.device.ABS_X, self.device.ABS_Y
        if abs(translate_x) > 0.95 or abs(translate_y) > 0.95:
            mujoco.mjv_moveCamera(
                self.scene.model,
                mujoco.mjtMouse.mjMOUSE_MOVE_H,
                -translate_x * 0.0005,
                -translate_y * 0.0005,
                self.scene.viewer.scn,
                self.scene.viewer.cam,
            )

        translate_z = self.device.BTN_TR - self.device.BTN_TL
        if abs(translate_z) > 0.1:
            mujoco.mjv_moveCamera(
                self.scene.model,
                mujoco.mjtMouse.mjMOUSE_MOVE_V,
                0,
                0.0005 * translate_z,
                self.scene.viewer.scn,
                self.scene.viewer.cam,
            )

        rotate_x, rotate_y = self.device.ABS_RX, self.device.ABS_RY
        if abs(rotate_x) > 0.95 or abs(rotate_y) > 0.95:
            mujoco.mjv_moveCamera(
                self.scene.model,
                mujoco.mjtMouse.mjMOUSE_ROTATE_V,
                rotate_x * 0.0002,
                rotate_y * 0.0002,
                self.scene.viewer.scn,
                self.scene.viewer.cam,
            )
        return True


class InteractiveTCPControllerBase(CartPosQuatImpedenceController):
    def __init__(self, scene: MjScene, robot: RobotBase, robot_config: dict):
        super().__init__()
        self.scene: MjScene = scene
        self.robot: RobotBase = robot
        self.indicator_name = getattr(robot, "name") + "_tcp_indicator"
        self.robot_config = robot_config
        self.scene.add_object(
            Box(
                self.indicator_name,
                robot_config["init_end_eff_pos"],
                robot_config["init_end_eff_quat"],
                rgba=[1, 0, 0, 0.0],
                static=True,
                visual_only=True,
            )
        )
        self.control_step = 1
        self.timer = 0

    def getControl(self, robot: RobotBase):
        if self.timer % self.control_step == 0:
            desired_pos, desired_quat = self.read_ctrl_pos(), self.read_ctrl_quat()
            # if self.timer % 100 == 0:
            #     print(desired_pos)
            # self.timer += 1
            desired_pos_local = robot._localize_cart_pos(desired_pos)
            desired_quat_local = robot._localize_cart_quat(desired_quat)
            self.setSetPoint(np.hstack((desired_pos_local, desired_quat_local)))
            # comment this line if not using gamepad
            self.scene.set_obj_pos_and_quat(
                desired_pos, desired_quat, obj_name=self.indicator_name
            )
        self.timer += 1
        return super().getControl(robot)

    def reset_robot(self):
        pos_local, quat_local = self.robot._localize_cart_coords(
            self.robot_config["init_end_eff_pos"],
            self.robot_config["init_end_eff_quat"],
        )
        self.scene.set_obj_pos_and_quat(
            pos_local, quat_local, obj_name=self.indicator_name
        )

    @abc.abstractmethod
    def read_ctrl_pos(self):
        pass

    @abc.abstractmethod
    def read_ctrl_quat(self):
        pass


class GamePadTCPController(InteractiveTCPControllerBase):
    def __init__(
        self, scene: Scene, robot: RobotBase, robot_config: dict, fix_rot=False
    ) -> None:
        self.pos_dampening = 5000.0
        self.rot_dampening = 5000.0
        self.ctrl_device: GamePad = GamePad()
        self.on_finger_control_triggered: bool = False

        self.viewController = MjViewGamePadController(scene, self.ctrl_device)
        super().__init__(scene, robot, robot_config)
        self.signal_list: list[BinarySingal] = list()
        self.finger_signal = BinarySingal(
            self.btn_x, init_state=robot.gripper_width > 0.078
        )
        self.signal_list.append(self.finger_signal)

    def getControl(self, robot: RobotBase):
        if self.viewController.controlViewCamera():
            return CartPosQuatImpedenceController.getControl(self, robot)

        for signal in self.signal_list:
            signal.update()

        if self.finger_signal.get_current_signal():
            if self.finger_signal.get_state():
                robot.open_fingers()
            else:
                robot.close_fingers(duration=0.0)

        if self.btn_y():
            self.scene.set_obj_quat(
                np.array([0.0, 1.0, 0.0, 0.0]), obj_name=self.indicator_name
            )
        return super().getControl(robot)

    def btn_a(self):
        return self.ctrl_device.BTN_SOUTH == 1

    def btn_b(self):
        return self.ctrl_device.BTN_EAST == 1

    def btn_x(self):
        return self.ctrl_device.BTN_NORTH == 1

    def btn_y(self):
        return self.ctrl_device.BTN_WEST == 1

    def reset(self):
        return self.ctrl_device.BTN_THUMBL == 1

    def start(self):
        return self.ctrl_device.BTN_START == 1

    def select(self):
        return self.ctrl_device.BTN_SELECT == 1

    def read_ctrl_pos(self):
        tcp_pos = self.scene.get_obj_pos(obj_name=self.indicator_name)
        azimuth, elevation = self.viewController.get_view_direction()
        x_signal, y_signal = self.ctrl_device.ABS_X, self.ctrl_device.ABS_Y
        if elevation > 0:
            y_signal = -y_signal
        pos_dir = (
            np.array(
                [
                    -y_signal * cos(azimuth) + x_signal * sin(azimuth),
                    -y_signal * sin(azimuth) - x_signal * cos(azimuth),
                    self.ctrl_device.BTN_TR - self.ctrl_device.BTN_TL,
                ]
            )
            / self.pos_dampening
        )
        pos_dir[np.abs(pos_dir) < 0.0001] = 0.0
        return pos_dir + tcp_pos

    def read_ctrl_quat(self):
        tcp_quat = self.scene.get_obj_quat(obj_name=self.indicator_name)
        azimuth, elevation = self.viewController.get_view_direction()
        rx_signal, ry_signal = self.ctrl_device.ABS_RX, self.ctrl_device.ABS_RY
        if elevation > 0:
            ry_signal = -ry_signal

        euler = (
            np.array(
                [
                    ry_signal * sin(azimuth) + rx_signal * cos(azimuth),
                    -ry_signal * cos(azimuth) + rx_signal * sin(azimuth),
                    (self.ctrl_device.ABS_RZ - self.ctrl_device.ABS_Z) * 3,
                ]
            )
            * np.pi
            / self.rot_dampening
        )
        euler[np.abs(euler) < 0.0001] = 0.0
        return geo_trans.quat_mul(geo_trans.euler2quat(euler), tcp_quat)


class ViveProMotionControllerTCPController(InteractiveTCPControllerBase):
    def __init__(self, scene: MjScene, robot: RobotBase, robot_config: dict):
        super().__init__(scene, robot, robot_config)
        self.device = triad_openvr()
        self.device.print_discovered_objects()
        self.T_r: np.ndarray = np.eye(4)
        self.position = robot_config["init_end_eff_pos"]
        self.quat = robot_config["init_end_eff_quat"]
        # self.origin: np.ndarray
        self.resetOrigin(self.position, self.quat)

    def getControl(self, robot: RobotBase):
        state = self.device.devices["controller_1"].get_controller_inputs()
        # print(state["trigger"])
        if state["trigger"] <= 0.99:
            robot.open_fingers()
        else:
            robot.close_fingers(duration=0.0)
        self.updateMotionControllerPose()
        # print(self.position, quat2euler(self.quat))
        # print(quat2euler(self.quat) * 180 / np.pi)
        return super().getControl(robot)

    def updateMotionControllerPose(self):
        position, rotation = self.getMotionControllerPose()
        if position is None:
            return
        # print(rotation)
        T_mc = np.eye(4)
        T_mc[:3, 3] = position
        T_mc[:3, :3] = euler2mat(degEuler2radEuler(rotation))
        # new_T = np.matmul(T_mc, self.T_r)
        new_T = np.matmul(self.T_r, T_mc)
        self.position = new_T[:3, 3]
        # self.position = [new_T[0, 3], new_T[1, 3], new_T[2, 3]]
        # self.position = [0.525, 0.0, 0.3]

        new_T = np.matmul(new_T, self.T_ef)
        # self.quat = mat2quat(new_T[:3, :3])
        new_T = new_T[:3, :3]
        new_T = np.matmul(self.R_T, new_T)
        self.quat = mat2quat(new_T)

        self.position = [-position[2], -position[0], position[1] + 0.4]
        rotation = [-rotation[0], -rotation[2], -rotation[1]]
        self.quat = quat_mul(
            np.array(euler2quat(degEuler2radEuler(rotation))), np.array([0, 1, 0, 0])
        )

    def resetOrigin(self, ef_pose=None, ef_quat=None):
        position, rotation = None, None
        while position is None:
            position, rotation = self.getMotionControllerPose()
        # T_mc = np.eye(4)
        # T_mc[:3, 3] = pose[:3]
        # T_mc[:3, :3] = euler2mat(degEuler2radEuler(pose[3:]))
        # print("T_mc", T_mc)

        T_mc = np.eye(4)
        T_mc[:3, 3] = position
        T_mc[:3, :3] = euler2mat(degEuler2radEuler(rotation))

        self.T_ef = np.eye(4)
        if ef_pose is None:
            ef_pose = self.robot.current_c_pos_global
            ef_quat = self.robot.current_c_quat_global
        self.T_ef[:3, 3] = ef_pose
        # self.T_ef[:3, :3] = quat2mat(ef_quat)
        self.T_ef[:3, :3] = quat2mat([1, 0, 0, 0])
        self.R_T = quat2mat(ef_quat)
        # print("T_ef", T_ef)
        # self.T_r = np.linalg.inv(T_mc)
        # self.T_r = np.matmul(np.linalg.inv(T_mc), T_ef)
        self.T_r = np.matmul(self.T_ef, np.linalg.inv(T_mc))
        # print("T_r", self.T_r)

    def getMotionControllerPose(self):
        pose = self.device.devices["controller_1"].get_pose_euler()
        if pose is None:
            return None, None
        position = pose[:3]
        # position = [-pose[1], pose[0], pose[2]]
        # rotation = [-pose[5], -pose[4], pose[3]]
        rotation = pose[3:]
        # rotation[1] = -rotation[1]
        return position, rotation

    def read_ctrl_pos(self):
        return self.position

    def read_ctrl_quat(self):
        return self.quat


class HoloLensTCPController(InteractiveTCPControllerBase):
    def __init__(
        self,
        scene: Scene,
        robot: RobotBase,
        robot_config: dict,
    ) -> None:
        super().__init__(scene, robot, robot_config)

    # def getControl(self, robot: RobotBase):
    #     self.scene.viewer.cam.lookat = self.scene.get_obj_pos(obj_name="user_head_pose")
    #     rot = quat2euler(self.scene.get_obj_quat(obj_name="user_head_pose"))
    #     self.scene.viewer.cam.distance = 0
    #     self.scene.viewer.cam.azimuth, self.scene.viewer.cam.elevation = -degrees(rot[2]), degrees(rot[1])
    #     return super().getControl(robot)

    def read_ctrl_pos(self):
        return self.scene.get_obj_pos(obj_name=self.indicator_name)

    def read_ctrl_quat(self):
        return self.scene.get_obj_quat(obj_name=self.indicator_name)


class VirtualRobotTCPController(HoloLensTCPController):
    pass


class HandTrackerTCPController(HoloLensTCPController):
    def __init__(
        self,
        scene: Scene,
        robot: RobotBase,
        robot_config,
        absolute_orientation=False,
    ) -> None:
        super().__init__(scene, robot, robot_config)

        self.position_kalman_filter = CartKalmanFilter(
            np.array(robot_config["init_end_eff_pos"])
        )
        # self.position_kalman_filter = CartKalmanFilter(np.zeros(3))
        # self.rotation_kalman_filter = CartKalmanFilter(np.zeros(3))

        self.rot_dampening = 200.0
        self.pos_dampening = 200.0

        if absolute_orientation:
            self.rot_dampening = 1.0

    def read_ctrl_pos(self):
        return self.position_kalman_filter.get_filtered(
            self.scene.get_obj_pos(obj_name=self.indicator_name)
        )

    def read_ctrl_quat(self):
        # eular_state = geo_trans.quat2euler(self.scene.get_obj_quat(obj_name=self.indicator_name))
        # return geo_trans.euler2quat(self.rotation_kalman_filter.get_filtered(eular_state))
        return self.scene.get_obj_quat(obj_name=self.indicator_name)


class VTController(JointPDController):
    def __init__(
        self,
        real_robot: SlRobot,
        vt_scene: Scene,
        goal_pos_fct,
        goal_vel_fct,
        robot_config: dict,
        goal_gripper_width_fct=None,
    ):
        super().__init__()
        self.real_robot = real_robot
        self.goal_pos_fct = goal_pos_fct
        self.goal_vel_fct = goal_vel_fct
        self.goal_gripper_width_fct = goal_gripper_width_fct
        self.robot_config = robot_config
        self.viewController = MjViewGamePadController(vt_scene)

    def getControl(self, robot: RobotBase):
        self.viewController.controlViewCamera()
        self.setSetPoint(
            desired_pos=self.goal_pos_fct(), desired_vel=self.goal_vel_fct()
        )
        if self.goal_gripper_width_fct is not None:
            if self.goal_gripper_width_fct() <= 0.075:
                robot.close_fingers(duration=0)  # Grasp
            else:
                robot.open_fingers()
        return super().getControl(robot)

    def reset_robot(self):
        real_robot_controller = self.real_robot.activeController
        pos_local, quat_local = self.real_robot._localize_cart_coords(
            self.robot_config["init_end_eff_pos"],
            self.robot_config["init_end_eff_quat"],
        )
        self.real_robot.gotoCartPositionAndQuat(
            pos_local, quat_local, duration=3, global_coord=True, block=True
        )
        self.real_robot.activeController = real_robot_controller


class RealRobotController(HumanController):
    def __init__(self, primary_robot: SlRobot, regularize=True):
        super().__init__(primary_robot, regularize)


class ReplayerController(JointPDController):
    def __init__(
        self,
        replay_data: dict[dict[list]],
        robot: RobotBase,
        downsample_steps: float = 50,
        initial_index=0,
    ):
        super().__init__()
        self.replay_data = replay_data
        self.timer = 0
        self.index = initial_index
        self.downsample_steps = downsample_steps
        self.robot = robot
        self.desired_joint_pos_array = np.array(replay_data["des_joint_pos"])
        self.desired_joint_vel_array = np.array(replay_data["des_joint_vel"])
        self.desired_joint_acc_array = np.array(replay_data["des_joint_acc"])
        self.gripper_width_array = np.array(replay_data["gripper_width"])
        self.sequence_length = len(self.desired_joint_pos_array)

    def getControl(self, robot: RobotBase):
        if self.timer % self.downsample_steps == 0:
            self.desired_joint_pos = self.desired_joint_pos_array[self.index]
            self.desired_joint_vel = self.desired_joint_vel_array[self.index]
            self.desired_joint_acc = self.desired_joint_acc_array[self.index]
            if self.gripper_width_array[self.index] <= 0.075:
                robot.close_fingers(duration=0)
            else:
                robot.open_fingers()
            self.index += 1
        self.timer += 1
        return super().getControl(robot)

    def reset_robot(self):
        self.robot.beam_to_joint_pos(self.desired_joint_pos_array[0])

    def isSequenceFinished(self):
        return self.index >= self.sequence_length
