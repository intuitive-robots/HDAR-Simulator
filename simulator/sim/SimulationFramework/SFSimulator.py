import abc
from typing import Dict
from alr_sim.sims.mj_beta import MjRobot
from alr_sim.sims.mj_beta import MjScene
from alr_sim.controllers import ControllerBase
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import MujocoObject
from simpub.sim.sf_publisher import SFPublisher
from simpub.core.net_manager import init_net_manager

from .SFRecorder import SFRecorder


class SFSimulator(abc.ABC):

    def __init__(self, task_name: str, record_mode=True):
        init_net_manager("192.168.0.117")
        self.task_name = task_name
        self.sim_factory = SimRepository.get_factory("mj_beta")
        self.mj_scene = self.create_scene()
        self.robot_dict = self.create_robots()
        self.controller_dict = self.create_controller()
        self.mj_scene.start()
        self.publisher = SFPublisher(self.mj_scene)
        for name, robot in self.robot_dict.items():
            self.controller_dict[name].executeController(
                robot, maxDuration=1000, block=False
            )
        self.record_mode = record_mode
        self.recorder = SFRecorder(
            task_name,
            self.object_dict,
            self.robot_dict,
            self.mj_scene,
            record_mode=self.record_mode
        )
        self.recorder.start_record()

    def create_scene(self) -> MjScene:
        self.object_dict = self.create_objects()
        return self.sim_factory.create_scene(
            object_list=self.object_dict.values(),
        )

    @abc.abstractmethod
    def create_robots(self) -> Dict[str, MjRobot]:
        raise NotImplementedError

    @abc.abstractmethod
    def create_controller(self) -> Dict[str, ControllerBase]:
        raise NotImplementedError

    @abc.abstractmethod
    def create_objects(self) -> Dict[str, MujocoObject]:
        raise NotImplementedError

    def before_step(self):
        pass

    def after_step(self):
        pass

    def run(self):
        # count = 0
        while True:
            self.before_step()
            self.mj_scene.next_step()
            if self.record_mode:
                self.recorder.record()
            self.after_step()
        #     count += 1
        #     if count > 1000:
        #         break
        # self.recorder.save_record()
