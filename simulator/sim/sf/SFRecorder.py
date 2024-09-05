from typing import Dict
import pickle
import os
from alr_sim.sims.mj_beta import MjRobot
from alr_sim.sims.mj_beta import MjScene
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import MujocoObject

from ..recorder import Recorder


class SFRecorder(Recorder):

    def __init__(
        self,
        task_name: str,
        objects_dict: Dict[str, MujocoObject],
        robot_dict: Dict[str, MjRobot],
        mj_scene: MjScene,
        save_root_path="./SFDemoData/",
        record_mode=False,
        record_step=10,
    ) -> None:
        super().__init__(task_name, save_root_path, record_mode, record_step)
        self.objects_dict = objects_dict
        self.robot_dict = robot_dict
        self.mj_scene = mj_scene
        self.data = []
        self.header = {
            "task": task_name,
            "record_step": record_step,
            "simulation_dt": mj_scene.dt,
        }

    def _record(self):
        current_frame = {}
        for name, robot in self.robot_dict.items():
            current_frame[name] = {
                "joint_pos": robot.current_j_pos,
                "joint_vel": robot.current_j_vel,
                "des_joint_pos": robot.des_joint_pos,
                "des_joint_vel": robot.des_joint_vel,
                "des_joint_acc": robot.des_joint_acc,
                "cart_pos": robot.current_c_pos,
                "cart_quat": robot.current_c_quat,
                "cart_vel": robot.current_c_vel,
                "cart_quat_vel": robot.current_c_quat_vel,
                "des_c_pos": robot.des_c_pos,
                "des_c_vel": robot.des_c_vel,
                "des_quat": robot.des_quat,
                "des_quat_vel": robot.des_quat_vel,
                "gripper_width": robot.gripper_width,
            }
        for name, obj in self.objects_dict.items():
            current_frame[name] = {
                "pos": self.mj_scene.get_obj_pos(obj_name=obj.name),
                "quat": self.mj_scene.get_obj_quat(obj_name=obj.name),
            }
        self.data.append(current_frame)

    def _start_record(self):
        self.data = []
        self.init_objects_state = {}
        for obj_name, obj in self.objects_dict.items():
            self.init_objects_state[obj_name] = {
                "pos": self.mj_scene.get_obj_pos(obj_name=obj.name),
                "quat": self.mj_scene.get_obj_quat(obj_name=obj.name),
            }
        self.init_robots_state = {}
        for robot_name, robot in self.robot_dict.items():
            self.init_robots_state[robot_name] = {
                "joint_pos": robot.current_j_pos,
            }

    def _save_record(self, file_name):
        # in case of data overwriting
        header = self.header
        init_object_state = self.init_objects_state
        init_robots_state = self.init_robots_state
        data = self.data
        with open(os.path.join(self.save_path, file_name), "wb") as f:
            pickle.dump(
                {
                    "header": header,
                    "init_objects_state": init_object_state,
                    "init_robots_state": init_robots_state,
                    "data": data,
                }, f)
        super()._save_record(file_name)
