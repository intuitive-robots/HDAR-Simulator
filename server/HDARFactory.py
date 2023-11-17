from abc import abstractmethod
from signal import raise_signal
from gym.spaces import Box as GymBox
import numpy as np
from typing import Dict, List
import yaml

from alr_sim.core import Scene

from alr_sim.sims.SimFactory import SimRepository
from alr_sim.controllers.Controller import ControllerBase
from alr_sim.sims.SimFactory import SimFactory
from alr_sim.sims.mj_beta import MjFactory, MjRobot, MjScene
from alr_sim.core.sim_object import SimObject
from alr_sim.sims.sl.SlRobot import SlRobot
from alr_sim.sims.sl.SlScene import SlScene
from alr_sim.sims.sl.SlFactory import SlFactory
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import YCBMujocoObject
from alr_sim.sims.universal_sim import PrimitiveObjects
from alr_sim.sims.universal_sim.PrimitiveObjects import PrimitiveObject

from alr_sim.utils import sim_path
from alr_sim.utils.geometric_transformation import euler2quat
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CustomMujocoObject

from sim_pub.sf import SFSimPubFactory
from sim_pub.sf.sf_simobj_publisher import SFObjectPublisher, SFRobotPubliser

from server.HDARController import *
from server.utils import *
from server.HDARModel import generate_HDARObj_from_dict
from task.HDARTaskBase import HDARTask
from server.utils import *


class HDARFactory(SFSimPubFactory):
    _repository: Dict[str, type[HDARTask]] = {}
    _mj_factory: MjFactory = SimRepository.get_factory("mj")
    _sl_factory: SlFactory = SimRepository.get_factory("sl")
    _sl_scene: SlScene = None

    _controller_map: Dict[str: ControllerBase] = {
        
    }

    def create_mj_robot(cls, mj_scene: MjScene, config: Dict):
        return cls._mj_factory.create_robot(
            mj_scene,
            num_DoF=7,
            base_position=config["base_position"],
            base_orientation=euler2quat(degList2radList(config["base_rotation"])),
            gravity_comp=True,
            clip_actions=False,
            root=sim_path.FRAMEWORK_DIR,
            xml_path=sim_path.sim_framework_path("./models/mj/robot/panda.xml"),
        )

    @classmethod
    def create_mj_scene(cls, obj_list):
        return cls._mj_factory.create_scene(
            object_list=obj_list,
            dt=0.001,
            render=Scene.RenderMode.HUMAN,
            # render=Scene.RenderMode.OFFSCREEN,
            surrounding=sim_path.sim_framework_path(
                "./models/mujoco/surroundings/kit_lab_surrounding2.xml"
            )
        )

    @classmethod
    def create_sl_scene(cls) -> SlScene:
        if cls._sl_scene is None:
            cls._sl_scene = cls._sl_factory.create_scene(skip_home=True)
        return cls._sl_scene

    @classmethod
    def create_sl_robot(cls, sl_scene, sl_config) -> SlRobot:
        return cls._sl_factory.create_robot(
            sl_scene,
            robot_name=sl_config["sl_robot_name"],
            backend_addr=sl_config["backend_addr"],
            local_addr=sl_config["local_addr"],
            gripper_actuation=True,
        )

    @classmethod
    def create_task(cls, config: Dict) -> HDARTask:
        task_name = config["task"]
        task_setting = read_yaml_file(f"./{task_name}.yaml")
        obj_dict = [cls.create_mj_obj(name, attr) for name, attr in task_setting["objects"]]
        scene = cls.create_mj_scene(obj_dict)
        task = cls._repository[task_name](obj_dict, scene)
        robot_dict = [cls.create_sim_robot()]
        cls.create_pub_from_setting
        return task

    @classmethod
    def create_mj_obj(cls, name: str, attr: Dict)-> List[SimObject]:
        if attr["source"] == "ycb":
            return YCBMujocoObject(
                obj_type=attr["type"],
                obj_name=name,
                init_pos=attr["init_pos"],
                init_quat=attr["init_quat"],
                static=attr.get("static", False),
                visual_only=attr.get("visual_only", False),
            )
        elif attr["source"] == "primitive_object":
            if attr["type"] == "Box":
                obj_class = PrimitiveObjects.Box
            elif attr["type"] == "Sphere":
                obj_class = PrimitiveObjects.Sphere
            elif attr["type"] == "Cylinder":
                obj_class = PrimitiveObjects.Cylinder
            else:
                raise TypeError
            return obj_class(
                name=name,
                init_pos=attr["init_pos"],
                init_quat=attr["init_quat"],
                mass=attr.get("mass", 0.5),
                size=attr["size"],
                rgba=attr["rgba"],
                static=attr.get("static", False),
                visual_only=attr.get("visual_only", False),
            )

    def create_pub(cls, obj_setting) -> SFObjectPublisher:
        pass

    @classmethod
    def create_controller(cls):
        pass

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

    def create_controller(robot_pub: SFRobotPubliser):
        pass