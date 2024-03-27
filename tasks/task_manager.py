import numpy as np
from typing import List, Dict
import yaml

from alr_sim.core import RobotBase, Scene
from alr_sim.sims.SimFactory import SimFactory
from alr_sim.core.sim_object import SimObject
from alr_sim.utils import sim_path
from alr_sim.utils.geometric_transformation import euler2quat

import utils, controllers
from server.hdar_model import generate_HDARObj_from_dict


_repository = {}

def get_task_setting(task_type: str):
    task_setting_path = f"tasks/task_setting/{task_type}.yaml"
    with open(task_setting_path, "r") as f:
        task_setting = yaml.load(f, Loader=yaml.FullLoader)
    return task_setting


def get_manager(task_type: str, sim_factory: SimFactory, dt: float):
    if task_type in _repository:
        return _repository[task_type](sim_factory=sim_factory, dt=dt)
    print("Using default task manager")
    return TaskManager(task_type=task_type, sim_factory=sim_factory, dt=dt)


class TaskManager:
    def __init__(self, task_type: str, sim_factory: SimFactory, dt: float) -> None:
        self.sim_factory = sim_factory
        self.object_dict: Dict[str, SimObject] = {}
        self.robot_dict: Dict[str, RobotBase] = {}
        self.dt = dt

        task_setting = get_task_setting(task_type)
        self.objects_config: Dict[str, dict] = task_setting["objects"]
        self.robots_config: Dict[str, dict] = task_setting["robots"]
        robots_interaction_config = utils.get_virtual_robot_config()
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
    
    def _create_robot(self, name, config):
        base_position = config["base_position"]
        base_quat = euler2quat(
            np.array(config["base_rotation"]) * np.pi / 180
        )
        model_name = config.get("type", "gripper_panda")
        robot = self.sim_factory.create_robot(
            self.scene,
            num_DoF=7,
            base_position=base_position,
            base_orientation=base_quat,
            gravity_comp=True,
            clip_actions=False,
            root=sim_path.FRAMEWORK_DIR,
            xml_path=f"./model/robot/{model_name}.xml",
        )
        robot.type = model_name
        robot.name = name
        robot.init_end_eff_pos = config["init_end_eff_pos"]
        robot.init_end_eff_quat = config["init_end_eff_quat"]
        robot.interaction_method = config["interaction_method"]
        return robot

    def create_robots(self):
        for robot_name, robot_config in self.robots_config.items():
            self.robot_dict[robot_name] = self._create_robot(robot_name, robot_config)

    def reset_objects(self):
        for obj_name, obj in self.object_dict.items():
            self.scene.set_obj_pos_and_quat(
                obj.init_pos, obj.init_quat, obj_name=obj_name
            )

    def reset_robots(self):
        for robot in self.get_robot_list():
            if hasattr(robot.activeController, "reset_robot"):
                robot.activeController.reset_robot()

    def _add_robot_attr():
        pass
