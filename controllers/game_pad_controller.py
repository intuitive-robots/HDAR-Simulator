from typing import List, Dict
from alr_sim.sims.mj_beta import MjScene
from .binary_signal import BinarySingal
from alr_sim.sims.mujoco.mj_interactive.devices.gamepad import GamePad
from .tcp_controller import InteractiveTCPControllerBase
from alr_sim.controllers.IKControllers import CartPosQuatImpedenceController
from alr_sim.sims.mj_beta import MjScene
import numpy as np
import mujoco
from math import sin, cos, radians

from alr_sim.core import Scene, RobotBase
from alr_sim.utils import geometric_transformation as geo_trans


class MjViewGamePadController:
    def __init__(self, scene: MjScene, device: GamePad = None) -> None:
        self.scene = scene
        self.device = device
        if device is None:
            self.device = GamePad()
        self.view_mode: bool = False
        self.on_view_mode_triggered: bool = False

        self.signal_list: List[BinarySingal] = []
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
        return False
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
        self.signal_list: List[BinarySingal] = list()
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