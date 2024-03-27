from .tcp_controller import InteractiveTCPControllerBase
from alr_sim.core import Scene, RobotBase
import numpy as np
from .cart_kalm_filter import CartKalmanFilter


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