from abc import abstractmethod
from gym.spaces import Box as GymBox
import numpy as np
from typing import Dict, List
import yaml

from alr_sim.core import RobotBase, Scene
from alr_sim.sims.mj_beta import MjScene
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.controllers.Controller import ControllerBase
from alr_sim.sims.SimFactory import SimFactory
from alr_sim.sims.mj_beta import MjFactory
from alr_sim.core.sim_object import SimObject
from alr_sim.sims.sl.SlRobot import SlRobot
from alr_sim.sims.sl.SlScene import SlScene
from alr_sim.sims.sl.SlFactory import SlFactory
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import YCBMujocoObject
from alr_sim.sims.universal_sim.PrimitiveObjects import PrimitiveObject

from alr_sim.utils import sim_path
from alr_sim.utils.geometric_transformation import euler2quat
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CustomMujocoObject

from sim_pub.sfmj.sfmj_simobj_publisher import SFObjectPublisher, SFRobotPubliser

from server.HDARFactory import HDARFactory
from hdar_server.HDARController import *
from hdar_server.utils import get_task_setting, get_hdar_config
from hdar_server.HDARModel import generate_HDARObj_from_dict


class HDARTask:

    def __init__(self, obj_dict: List[SimObject], mj_scene: MjScene) -> None:
        self.obj_pub_dict: Dict[str, SFObjectPublisher] = {}
        self.robot_pub_dict: Dict[str, SFRobotPubliser] = {}
        self.initialize_scene()

    def initialize_scene(self):
        task_setting = HDARFactory.read_yaml_file()
        
        for obj_name, setting in self.obj_pub_dict["objects"]:        
            self.obj_pub_dict[obj_name] = HDARFactory.create_pub_from_setting(setting)
        obj_list = [item.sim_obj for item in self.obj_pub_dict.values()]
        self.sim_scene = HDARFactory.create_sim_scene(obj_list)
        for robot_name, setting in task_setting["robots"]:
            self.robot_pub_dict[robot_name]
            HDARFactory.create_publisher()
        self.scene_list: List[Scene] = [self.sim_scene]

    def reset_tasks(self):
        for obj_name, obj_pub in self.obj_pub_dict.items():
            init_pos = obj_pub.param_dict["init_pos"]
            init_quat = obj_pub.param_dict["init_quat"]
            self.sim_scene.set_obj_pos_and_quat(init_pos, init_quat, obj_name=obj_name)

    def reset_robots(self):
        pass

    def next_step(self):
        for scene in self.scene_list:
            scene.next_step()

    def set_up_controllers(self):
        for robot_name, pub in self.robot_pub_dict.items():
            pass

    def start_simulation(self):
        for scene in self.scene_list:
            scene.start()
        for robot, controller in zip(self.robot_list, self.controller_list):
            controller.executeController(robot, maxDuration=1000, block=False)

    @abstractmethod
    def is_task_finished(self):
        raise NotImplementedError


