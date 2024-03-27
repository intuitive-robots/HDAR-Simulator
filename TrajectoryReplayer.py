import pickle
import time
from enum import Flag, auto, Enum
import numpy as np

# from alr_sim.controllers.Controller import ControllerBase
from alr_sim.core import Scene, RobotBase
from alr_sim.core.sim_object import SimObject
from alr_sim.controllers.Controller import ControllerBase
from alr_sim.sims.SimFactory import SimRepository


from server.UnityHDRecorder import UnityHDRecorder
from server.UnityStreamer import UnityStreamer
from server.HDARController import *
from task.TaskManager import TaskManager


class TrajectoryReplayer:
    def __init__(self, data_path: str, task_type: str) -> None:
        data = np.load(data_path, allow_pickle=True)
        self.data = data
        # virtual twin
        self.vt_sim_factory = SimRepository.get_factory("mj_beta")
        self.scene_manager: TaskManager = TaskManager.get_manager(task_type)
        self.scene_manager.create_task(self.vt_sim_factory)

        self.vt_scene: Scene = self.scene_manager.get_scene()
        self.vt_object_dict = self.scene_manager.get_object_dict()
        self.vt_robot_dict = self.scene_manager.get_robot_dict()
        self.vt_controller_dict: dict[str, ReplayerController] = dict()

        for robot_name in self.scene_manager.robots_config.keys():
            vt_robot = self.vt_robot_dict[robot_name]
            vt_controller = ReplayerController(
                replay_data=data["robot"],
                robot=vt_robot,
                downsample_steps=50,
            )
            self.vt_controller_dict[robot_name] = vt_controller

        # just for hri paper
        self.streamer = UnityStreamer(
            self.vt_scene,
            [],
            [],
        )
        self.interaction_object_list = self.streamer.interaction_object_list

        self.vt_scene.start()

        # Update the object pose from data
        for obj_name, obj_data in data.items():
            if obj_name == "robot" or obj_name == "context":
                continue
            self.vt_scene.set_obj_pos_and_quat(
                obj_data["pos"][0],
                obj_data["orientation"][0],
                obj_name=obj_name,
            )

        for robot, controller in zip(
            self.vt_robot_dict.values(), self.vt_controller_dict.values()
        ):
            controller.reset_robot()
            controller.executeController(robot, maxDuration=1000, block=False)

    def run(self):
        t_index = 0
        while True:
            if t_index % 50 == 0:
                index = t_index // 50
                for obj in self.interaction_object_list:
                    obj_data = self.data[obj.name]
                    self.vt_scene.set_obj_pos_and_quat(
                        obj_data["pos"][index],
                        obj_data["orientation"][index],
                        obj,
                    )
            t_index += 1
            for controller in self.vt_controller_dict.values():
                if controller.isSequenceFinished():
                    return
            self.vt_scene.next_step()


if __name__ == "__main__":
    replayer = TrajectoryReplayer(
        # data_path="/home/xinkai/Desktop/LfD_Use_Study/004/AR_RR/practical_manipulation_2023_08_07_14_45_19/test001.pkl",
        # data_path="/home/xinkai/Desktop/LfD_Use_Study/004/Handtracking/practical_manipulation_2023_08_07_16_13_02/test001.pkl",
        # data_path="/home/xinkai/Desktop/LfD_Use_Study/004/AR_RR/cube_stacking_2023_08_07_14_29_16/test001.pkl",
        # data_path="/home/xinkai/Desktop/LfD_Use_Study/004/Handtracking/practical_manipulation_2023_08_07_16_13_02/practical_manipulation_000.pkl",
        # data_path="/home/xinkai/SimulationFramework/ARHumanDemoData/cup_stacking_2023_08_11_11_18_53/cup_stacking_000.pkl",
        data_path="/home/xinkai/SimulationFramework/ARHumanDemoData/box_stacking_2023_08_28_13_51_45/box_stacking_000.pkl",
        # task_type="warm_up",
        task_type="box_stacking",
        # task_type="cup_stacking",
        # task_type="practical_manipulation",
    )
    replayer.run()
