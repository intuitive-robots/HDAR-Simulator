import os
import pickle
from datetime import datetime

from alr_sim.core import Scene, RobotBase
from alr_sim.core.sim_object import SimObject
from alr_sim.core.logger import ObjectLogger
from alr_sim.utils.sim_path import sim_framework_path

from .UnityStreamer import UnityStreamer

import multiprocessing as mp

from typing import List


class UnityHDRecorder:
    def __init__(
        self,
        scene: Scene,
        obj_list: List[SimObject],
        task_type,
        streamer: UnityStreamer,
        manager,
        save_root_path=None,
        record_mode=False,
        downsample_steps=1,
    ) -> None:
        if save_root_path is None:
            save_root_path = os.path.join(
                sim_framework_path(),
                "ARHumanDemoData/",
            )
        self.save_root_path = save_root_path
        self.record_mode = record_mode
        self.scene = scene
        self.robots: RobotBase = scene.robots
        self.obj_list = obj_list
        self.task_type = task_type
        self.downsample_steps = downsample_steps
        obj_loggers = dict()
        for obj in self.obj_list:
            obj_logger = ObjectLogger(scene, obj)
            scene.add_logger(obj_logger)
            obj_loggers[obj.name] = obj_logger

        # for obj in streamer.interaction_object_list:
        #     obj_logger = ObjectLogger(scene, obj)
        #     scene.add_logger(obj_logger)
        #     obj_loggers[obj.name] = obj_logger

        self.obj_loggers = obj_loggers
        self.log_counter = 0
        self.save_path = os.path.join(
            self.save_root_path,
            "{}_{}".format(task_type, datetime.now().strftime("%Y_%m_%d_%H_%M_%S")),
        )
        # flags
        self.on_logging: bool = False
        self.manager = manager

    def start_record(self):
        if not self.record_mode or self.on_logging:
            return
        print("Start recording")
        self.on_logging = True
        self.scene.start_logging()

    def stop_record(self):
        if not self.record_mode or not self.on_logging:
            return
        print("Stop recording")
        self.on_logging = False
        self.scene.stop_logging()

    def save_record(self):
        if not self.record_mode:
            return
        self.stop_record()
        file_name = "{}_{:03d}.pkl".format(self.task_type, self.log_counter)
        self.log_counter += 1
        mp.Process(target=self._save_record, kwargs={"file_name": file_name}).start()

    def _save_record(self, file_name):
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        state_dict = dict()
        for robot in self.robots:
            # state_dict["robot"] = robot.robot_logger.log_dict_full
            robot_state_dict = dict()
            robot_log = robot.robot_logger.log_dict_full
            for attr, data in robot_log.items():
                robot_state_dict[attr] = data[:: self.downsample_steps]
            state_dict["robot"] = robot_state_dict

        for obj_name, obj_logger in self.obj_loggers.items():
            # state_dict[obj_name] = obj_logger.log_dict_full
            obj_state_dict = dict()
            obj_log = obj_logger.log_dict_full
            for attr, data in obj_log.items():
                obj_state_dict[attr] = data[:: self.downsample_steps]
            state_dict[obj_name] = obj_state_dict

        # just for David's task
        if hasattr(self.manager, "context"):
            state_dict["context"] = self.manager.context
        with open(os.path.join(self.save_path, file_name), "wb") as f:
            pickle.dump(state_dict, f)
        print(f"The file has been saved to {file_name}")
