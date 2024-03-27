import numpy as np

from alr_sim.core import RobotBase, Scene
from alr_sim.sims.SimFactory import SimFactory
from alr_sim.core.sim_object import SimObject

from alr_sim.utils import sim_path
from alr_sim.utils.geometric_transformation import euler2quat

from controllers import *
from server.utils import get_hdar_config
from server.HDARModel import generate_HDARObj_from_dict

from typing import List, Dict
import yaml


def get_task_setting(task_type: str):
    task_setting_path = f"task/task_setting/{task_type}.yaml"
    with open(task_setting_path, "r") as f:
        task_setting = yaml.load(f, Loader=yaml.FullLoader)
    return task_setting


class TaskManager:
    _repository = {}

    @classmethod
    def get_manager(cls, task_type: str, sim_factory: SimFactory, dt: float):
        if task_type in cls._repository:
            return cls._repository[task_type](sim_factory=sim_factory, dt=dt)
        print("Using default task manager")
        return TaskManager(task_type=task_type, sim_factory=sim_factory, dt=dt)

    def __init__(self, task_type: str, sim_factory: SimFactory, dt: float) -> None:
        self.sim_factory = sim_factory
        self.object_dict: Dict[str, SimObject] = {}
        self.robot_dict: Dict[str, RobotBase] = {}
        self.dt = dt

        task_setting = get_task_setting(task_type)
        self.objects_config: Dict[str, dict] = task_setting["objects"]
        self.robots_config: Dict[str, dict] = task_setting["robots"]
        robots_interaction_config = get_hdar_config("RobotConfig")
        for robot_name, config in self.robots_config.items():
            config.update(robots_interaction_config[robot_name])

    def create_task(self):
        self.create_objects()
        self.scene: Scene = self.sim_factory.create_scene(
            object_list=self.get_object_list(),
            dt=self.dt,
            render=Scene.RenderMode.HUMAN,
            # render=Scene.RenderMode.OFFSCREEN,
            surrounding=sim_path.sim_framework_path(
                "./models/mujoco/surroundings/kit_lab_surrounding2.xml"
            ),
        )
        self.create_robots()

    def get_object_dict(self) -> Dict[str, SimObject]:
        return self.object_dict

    def get_robot_dict(self) -> Dict[str, RobotBase]:
        return self.robot_dict

    def get_object_list(self) -> List[SimObject]:
        return list(self.object_dict.values())

    def get_robot_list(self) -> List[RobotBase]:
        return list(self.robot_dict.values())

    def get_scene(self) -> Scene:
        return self.scene

    def is_task_finished(self):
        return False

    def create_objects(self):
        for obj_name, obj_config in self.objects_config.items():
            new_obj = generate_HDARObj_from_dict(obj_name, obj_config)
            self.object_dict[obj_name] = new_obj

    def create_robots(self):
        for robot_name, robot_config in self.robots_config.items():
            base_position = robot_config["base_position"]
            base_quat = euler2quat(
                np.array(robot_config["base_rotation"]) * np.pi / 180
            )
            model_name = robot_config.get("type", "gripper_panda")
            new_robot = self.sim_factory.create_robot(
                self.scene,
                num_DoF=7,
                base_position=base_position,
                base_orientation=base_quat,
                gravity_comp=True,
                clip_actions=False,
                root=sim_path.FRAMEWORK_DIR,
                xml_path=f"./model/robot/{model_name}.xml",
            )
            new_robot.type = model_name
            new_robot.name = robot_name
            new_robot.init_end_eff_pos = robot_config["init_end_eff_pos"]
            new_robot.init_end_eff_quat = robot_config["init_end_eff_quat"]
            new_robot.interaction_method = robot_config["interaction_method"]
            self.robot_dict[robot_name] = new_robot

    def reset_objects(self):
        for obj_name, obj in self.object_dict.items():
            self.scene.set_obj_pos_and_quat(
                obj.init_pos, obj.init_quat, obj_name=obj_name
            )

    def reset_robots(self):
        for robot in self.get_robot_list():
            if robot.activeController is not RealRobotController:
                robot.activeController.reset_robot()

    def _add_robot_attr():
        pass
