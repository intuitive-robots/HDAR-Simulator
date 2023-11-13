from abc import abstractmethod
from gym.spaces import Box as GymBox
import numpy as np

from alr_sim.core import RobotBase, Scene
from alr_sim.controllers.Controller import ControllerBase
from alr_sim.sims.SimFactory import SimFactory
from alr_sim.core.sim_object import SimObject
from alr_sim.sims.sl.SlRobot import SlRobot
from alr_sim.sims.sl.SlScene import SlScene

from alr_sim.utils import sim_path
from alr_sim.utils.geometric_transformation import euler2quat

from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CustomMujocoObject

from hdar_server.HDARController import *
from hdar_server.utils import get_task_setting, get_hdar_config
from hdar_server.HDARModel import generate_HDARObj_from_dict


class TaskManager:

    _repository = {}

    @classmethod
    def get_manager(cls, task_type: str):
        return cls._repository[task_type]()

    def __init__(self, task_type) -> None:
        self.object_list: list[SimObject]
        self.object_dict: dict[str, SimObject] = {}
        self.robot_dict: dict[str, RobotBase] = {}

        task_setting = get_task_setting(task_type)
        self.objects_config: dict[str, dict] = task_setting["objects"]
        self.robots_config: dict[str, dict] = task_setting["robots"]
        robots_interaction_config = get_hdar_config("RobotConfig")
        for robot_name, config in self.robots_config.items():
            config.update(robots_interaction_config[robot_name])

    def create_task(self, sim_factory: SimFactory):
        self.create_objects()
        self.scene: Scene = sim_factory.create_scene(
            object_list=self.get_object_list(),
            dt=0.001,
            render=Scene.RenderMode.HUMAN,
            # render=Scene.RenderMode.OFFSCREEN,
            surrounding=sim_path.sim_framework_path(
                "./models/mujoco/surroundings/kit_lab_surrounding2.xml"
            ),
        )
        self.create_robots(sim_factory)

    def get_object_dict(self) -> dict[str, SimObject]:
        return self.object_dict

    def get_robot_dict(self) -> dict[str, RobotBase]:
        return self.robot_dict

    def get_object_list(self) -> list[SimObject]:
        return self.object_dict.values()

    def get_robot_list(self) -> list[RobotBase]:
        return self.robot_dict.values()

    def get_scene(self) -> Scene:
        return self.scene

    @abstractmethod
    def is_task_finished(self):
        raise NotImplementedError

    @abstractmethod
    def create_objects(self):
        for obj_name, obj_config in self.objects_config.items():
            new_obj = generate_HDARObj_from_dict(obj_name, obj_config)
            self.object_dict[obj_name] = new_obj

    @abstractmethod
    def create_robots(self, sim_factory: SimFactory):
        for robot_name, robot_config in self.robots_config.items():
            new_robot = sim_factory.create_robot(
                self.scene,
                num_DoF=7,
                base_position=robot_config["base_position"],
                base_orientation=euler2quat(
                    np.array(robot_config["base_rotation"]) * np.pi / 180
                ),
                gravity_comp=True,
                clip_actions=False,
                root=sim_path.FRAMEWORK_DIR,
                xml_path=sim_path.sim_framework_path("./models/mj/robot/panda.xml"),
            )
            setattr(new_robot, "name", robot_name)
            setattr(new_robot, "init_end_eff_pos", robot_config["init_end_eff_pos"])
            setattr(new_robot, "init_end_eff_quat", robot_config["init_end_eff_quat"])
            setattr(new_robot, "interaction_method", robot_config["interaction_method"])
            self.robot_dict[robot_name] = new_robot

    @abstractmethod
    def reset_objects(self):
        for obj_name, obj in self.object_dict.items():
            self.scene.set_obj_pos_and_quat(
                obj.init_pos, obj.init_quat, obj_name=obj_name
            )

    # def reset_robots(self):
    #     for robot in self.robot_list:
    #         robot.activeController.reset()

    def _add_robot_attr():
        pass
