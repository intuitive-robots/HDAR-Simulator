import numpy as np
from math import radians

from alr_sim.core.Scene import Scene


def degEuler2radEuler(degEuler):
    return [radians(x) for x in degEuler]


# def str2list(s: str):
#     return [ord(c) for c in list(s)]


# def objects_distance(obj1_name: str, obj2_name: str, scene: Scene):
#     pos1 = scene.get_obj_pos(obj_name=obj1_name)
#     pos2 = scene.get_obj_pos(obj_name=obj2_name)
#     return np.linalg.norm(pos1 - pos2)


# def objects_xy_distance(obj1_name: str, obj2_name: str, scene: Scene):
#     pos1 = scene.get_obj_pos(obj_name=obj1_name)[0:2]
#     pos2 = scene.get_obj_pos(obj_name=obj2_name)[0:2]
#     return np.linalg.norm(pos1 - pos2)


# def limit_number(num, min_num, max_num):
#     return min(max(num, min_num), max_num)

# def constrain_in_workspace(pose):
#     pose[0] = limit_number(pose[0], )
