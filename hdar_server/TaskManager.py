# from abc import abstractmethod
# from gym.spaces import Box as GymBox
# import numpy as np

# from alr_sim.core import RobotBase, Scene
# from alr_sim.controllers.Controller import ControllerBase
# from alr_sim.sims.SimFactory import SimFactory
# from alr_sim.core.sim_object import SimObject
# from alr_sim.sims.sl.SlRobot import SlRobot
# from alr_sim.sims.sl.SlScene import SlScene

# from alr_sim.utils import sim_path
# from alr_sim.utils.geometric_transformation import euler2quat

# from .utils import get_task_objects_setting, get_task_robots_setting
# from .utils import objects_distance, objects_xy_distance
# from .model import _object_map

# from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CustomMujocoObject
# import os

# from hdar_server.HDARController import *


# class TaskManager:

#     _repository = {}

#     @classmethod
#     def get_manager(cls, task_type):
#         return cls._repository[task_type]()

#     def __init__(self) -> None:
#         self.robot_list: list[RobotBase] = []
#         self.controller_list: list[ControllerBase] = []

#         self.objects_list_yaml: list[dict]
#         self.robots_list_yaml: list[dict]

#     def create_task(self, task_type: str, sim_factory: SimFactory):
#         self.objects_list_yaml: list[dict] = get_task_objects_setting(task_type)
#         self.robots_list_yaml: list[dict] = get_task_robots_setting(task_type)

#         object_list = list()
#         for obj_dict in self.objects_list_yaml:
#             obj_class = _object_map[obj_dict["type"]]
#             obj_dict.pop("type")
#             object_list.append(obj_class(**obj_dict))
#         self.object_list = object_list

#         self.scene: Scene = sim_factory.create_scene(
#             object_list=self.object_list,
#             dt=0.001,
#             render=Scene.RenderMode.HUMAN,
#             # render=Scene.RenderMode.OFFSCREEN,
#             surrounding=sim_path.sim_framework_path(
#                 "./models/mujoco/surroundings/kit_lab_surrounding2.xml"
#             ),
#         )

#         for robot_config in self.robots_list_yaml:
#             if robot_config["type"] == "gripper_panda":
#                 xml_path = sim_path.sim_framework_path("./models/mj/robot/panda.xml")
#             elif robot_config["type"] == "rod_panda":
#                 xml_path = sim_path.sim_framework_path(
#                     "./models/mj/robot/panda_rod.xml"
#                 )
#             else:
#                 xml_path = None
#             robot = sim_factory.create_robot(
#                 self.scene,
#                 num_DoF=7,
#                 base_position=robot_config["base_position"],
#                 base_orientation=euler2quat(
#                     np.array(robot_config["base_rotation"]) * np.pi / 180
#                 ),
#                 gravity_comp=True,
#                 clip_actions=False,
#                 root=sim_path.FRAMEWORK_DIR,
#                 xml_path=xml_path,
#             )
#             robot.type = robot_config["type"]
#             robot.name = robot_config["name"]
#             robot.interaction_method = robot_config["interaction_method"]
#             self.robot_list.append(robot)

#     def get_object_list(self) -> list[SimObject]:
#         return self.object_list

#     def get_robot_list(self) -> list[RobotBase]:
#         return self.robot_list

#     def get_scene(self) -> Scene:
#         return self.scene

#     @abstractmethod
#     def is_task_finished(self):
#         raise NotImplementedError

#     def reset_task(self):
#         for obj_yaml, obj in zip(self.objects_list_yaml, self.object_list):
#             new_pos = obj_yaml["init_pos"]
#             new_quat = obj_yaml["init_quat"]
#             self.scene.set_obj_pos_and_quat(new_pos, new_quat, obj_name=obj.name)


# class GraspTestManager(TaskManager):
#     def __init__(self) -> None:
#         super().__init__()

#     def is_task_finished(self):
#         # return (
#         #     objects_distance("push_box1", "target_box_1", self.scene) < 0.05
#         #     and objects_distance("push_box2", "target_box_2", self.scene) < 0.05
#         # )
#         False


# class BoxPushManager(TaskManager):
#     def __init__(self) -> None:
#         super(BoxPushManager).__init__()

#     def is_task_finished(self):
#         # return (
#         #     objects_distance("push_box1", "target_box_1", self.scene) < 0.05
#         #     and objects_distance("push_box2", "target_box_2", self.scene) < 0.05
#         # )
#         False


# class CubeStackingManager(TaskManager):
#     def __init__(self) -> None:
#         super().__init__()

#         np.random.seed(42)
#         self.red_space = GymBox(
#             low=np.array([0.35, -0.25, -90]),
#             high=np.array([0.45, -0.15, 90]),  # , seed=seed
#         )

#         self.green_space = GymBox(
#             low=np.array([0.35, -0.1, -90]), high=np.array([0.45, 0, 90])  # , seed=seed
#         )

#         self.blue_space = GymBox(
#             low=np.array([0.55, -0.2, -90]), high=np.array([0.6, 0, 90])  # , seed=seed
#         )

#         self.target_space = GymBox(
#             low=np.array([0.4, 0.15, -90]),
#             high=np.array([0.6, 0.25, 90]),  # , seed=seed
#         )

#     def reset_task(self, random=True, context=None):
#         super().reset_task()
#         for robot in self.robot_list:
#             robot.open_fingers()

#         if random:
#             self.context = self.sample()
#         else:
#             self.context = context

#         self.set_context(self.context)

#     def is_task_finished(self):
#         # return (
#         #     objects_xy_distance("target_box", "blue_box", self.scene) < 0.01
#         #     and objects_xy_distance("target_box", "blue_box", self.scene) < 0.01
#         # )
#         return False

#     def sample(self):

#         pos_1 = self.red_space.sample()
#         angle_1 = [0, 0, pos_1[-1] * np.pi / 180]
#         quat_1 = euler2quat(angle_1)

#         pos_2 = self.green_space.sample()
#         angle_2 = [0, 0, pos_2[-1] * np.pi / 180]
#         quat_2 = euler2quat(angle_2)

#         pos_3 = self.blue_space.sample()
#         angle_3 = [0, 0, pos_3[-1] * np.pi / 180]
#         quat_3 = euler2quat(angle_3)

#         pos_4 = self.target_space.sample()
#         angle_4 = [0, 0, pos_4[-1] * np.pi / 180]
#         quat_4 = euler2quat(angle_4)

#         return [pos_1, quat_1], [pos_2, quat_2], [pos_3, quat_3], [pos_4, quat_4]

#     def set_context(self, context):

#         red_pos = context[0][0]
#         red_quat = context[0][1]

#         green_pos = context[1][0]
#         green_quat = context[1][1]

#         blue_pos = context[2][0]
#         blue_quat = context[2][1]

#         target_pos = context[3][0]
#         target_quat = context[3][1]

#         self.scene.set_obj_pos_and_quat(
#             [red_pos[0], red_pos[1], 0],
#             red_quat,
#             obj_name="red_box",
#         )

#         self.scene.set_obj_pos_and_quat(
#             [green_pos[0], green_pos[1], 0],
#             green_quat,
#             obj_name="green_box",
#         )

#         self.scene.set_obj_pos_and_quat(
#             [blue_pos[0], blue_pos[1], 0],
#             blue_quat,
#             obj_name="blue_box",
#         )

#         # self.scene.set_obj_pos_and_quat(
#         #     [target_pos[0], target_pos[1], 0],
#         #     [0, 1, 0, 0],
#         #     obj_name="target_box",
#         # )


# class StackingManager(TaskManager):
#     def __init__(self) -> None:
#         super().__init__()
#         np.random.seed(42)

#         self.square_box_space = GymBox(
#             low=np.array([0.3, -0.25, -90]),
#             high=np.array([0.45, -0.1, 90]),  # , seed=seed
#         )

#         self.round_box_space = GymBox(
#             low=np.array([0.55, -0.25, -90]),
#             high=np.array([0.7, -0.1, 90]),  # , seed=seed
#         )

#         # index = 0, push from inside
#         # index = 1, push from outside

#     def reset_task(self, random=True, context=None):

#         if random:
#             self.context = self.sample()
#         else:
#             self.context = context

#         self.set_context(self.context)

#     def sample(self):

#         square_pos = self.square_box_space.sample()
#         goal_angle = [0, 0, square_pos[-1] * np.pi / 180]
#         # goal_angle = [square_pos[-1] * np.pi / 180, 0, 0]
#         square_quat = euler2quat(goal_angle)

#         round_box_pos = self.round_box_space.sample()
#         round_box_angle = [0, 0, round_box_pos[-1] * np.pi / 180]
#         # round_box_angle = [round_box_pos[-1] * np.pi / 180, 0, 0]
#         round_box_quat = euler2quat(round_box_angle)

#         return [square_pos, square_quat, round_box_pos, round_box_quat]

#     def set_context(self, context):

#         square_pos, square_quat, round_box_pos, round_box_quat = context
#         # print(square_pos, square_quat, round_box_pos, round_box_quat)
#         # self.scene.set_obj_pos_and_quat(
#         #     [square_pos[0], square_pos[1], 0.05],
#         #     square_quat,
#         #     # [1, 0, 0, 0],
#         #     obj_name="square_box",
#         # )

#         self.scene.set_obj_pos_and_quat(
#             [round_box_pos[0], round_box_pos[1], 0.05],
#             round_box_quat,
#             # [1, 0, 0, 0],
#             # obj_name="round_box",
#             obj_name="cup",
#         )

#     def is_task_finished(self):
#         return False


# class BimanualTestManager(TaskManager):
#     def __init__(self) -> None:
#         super().__init__()

#     def is_task_finished(self):
#         # return (
#         #     objects_distance("push_box1", "target_box_1", self.scene) < 0.05
#         #     and objects_distance("push_box2", "target_box_2", self.scene) < 0.05
#         # )
#         False


# TaskManager._repository = {
#     "grasp_test": GraspTestManager,
#     "box_push": BoxPushManager,
#     "stacking": StackingManager,
#     "cube_stacking": CubeStackingManager,
#     "bimanual_test": BimanualTestManager,
# }
