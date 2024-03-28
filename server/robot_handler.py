from alr_sim.core.Scene import Scene
from alr_sim.core.Robots import RobotBase

from utils.unity_utils import mj2unity_pos, mj2unity_quat


class RobotHandler:
    count = 0

    def __init__(self, robot: RobotBase, scene: Scene) -> None:
        self.robot = robot
        self.scene = scene

        if hasattr(robot, "name"):
            self.name = getattr(robot, "name")
        else:
            self.name = f"panda{RobotHandler.count}"
            RobotHandler.count += 1
        self.type = getattr(robot, "type", "gripper_panda")

    def get_robot_param_dict(self) -> dict:
        param_date = {}
        param_date["pos"] = list(mj2unity_pos(self.robot.base_position))
        param_date["rot"] = list(mj2unity_quat(self.robot.base_orientation))
        joints = list(self.robot.current_j_pos)
        joints.extend([self.robot.gripper_width / 2, self.robot.gripper_width / 2])
        param_date["joints"] = joints
        return {
            "attr": {
                "type": self.type,
                "interaction_method": getattr(self.robot, "interaction_method"),
            },
            "data": param_date,
        }

    def get_robot_state_dict(self) -> dict:
        joints = list(self.robot.current_j_pos)
        joints.extend([self.robot.gripper_width / 2, self.robot.gripper_width / 2])
        return {
            "data": {
                "joints": joints,
            }
        }