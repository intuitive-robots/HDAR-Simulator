from ast import List
import os
import yaml
import numpy as np
from math import radians

from alr_sim.sims.universal_sim.PrimitiveObjects import PrimitiveObject
from alr_sim.core.Scene import Scene
from alr_sim.utils.sim_path import sim_framework_path


def degList2radList(degEuler: List[float]) -> List[float]:
    return [radians(x) for x in degEuler]


def str2list(s: str):
    return [ord(c) for c in list(s)]


def unity2mj_pos(pos):
    return [-pos[1], pos[2], pos[0]]


def unity2mj_quat(quat):
    # note that the order is "[x, y, z, w]"
    return [quat[2], -quat[3], -quat[1], quat[0]]


def read_yaml_file(config_path):
    with open(config_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    return config

def get_task_setting(task_type: str):
    task_setting_path = sim_framework_path(
        "demos",
        "unity",
        "human_demostration_ar",
        "hdar_task",
        "task_setting",
        f"{task_type}.yaml",
    )
    with open(task_setting_path, "r") as f:
        task_setting = yaml.load(f, Loader=yaml.FullLoader)
    return task_setting


def get_hdar_config(config_name):
    qr_config_path = sim_framework_path(
        "demos", "unity", "human_demostration_ar", "config.yaml"
    )
    with open(qr_config_path, "r") as f:
        task_setting = yaml.load(f, Loader=yaml.FullLoader)
    return task_setting[config_name]


def objects_distance(obj1_name: str, obj2_name: str, scene: Scene):
    pos1 = scene.get_obj_pos(obj_name=obj1_name)
    pos2 = scene.get_obj_pos(obj_name=obj2_name)
    return np.linalg.norm(pos1 - pos2)


def objects_xy_distance(obj1_name: str, obj2_name: str, scene: Scene):
    pos1 = scene.get_obj_pos(obj_name=obj1_name)[0:2]
    pos2 = scene.get_obj_pos(obj_name=obj2_name)[0:2]
    return np.linalg.norm(pos1 - pos2)


# def limit_number(num, min_num, max_num):
#     return min(max(num, min_num), max_num)

# def constrain_in_workspace(pose):
#     pose[0] = limit_number(pose[0], )
