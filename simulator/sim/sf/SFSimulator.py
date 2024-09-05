import abc
from typing import Dict
from alr_sim.sims.mj_beta import MjRobot
from alr_sim.sims.mj_beta import MjScene
from alr_sim.controllers import ControllerBase
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import MujocoObject
from simpub.sim.sf_publisher import SFPublisher

from .SFRecorder import SFRecorder


class SFSimulator(abc.ABC):

    def __init__(
        self,
        task_name: str,
        record_mode=True,
        host_address="127.0.0.1",
    ):
        # TODO: why init_net cannot be started firstly?
        # TODO: Because the broadcast is running and unity send message once it is connected
        # TODO: but the service is not ready yet
        # TODO: Think about one way to fix it.
        # init_net_manager(host_address)
        self.task_name = task_name
        self.sim_factory = SimRepository.get_factory("mj_beta")
        self.mj_scene = self.create_scene()
        self.robot_dict = self.create_robots()
        self.mj_scene.start()
        self.publisher = SFPublisher(self.mj_scene, host_address)
        self.record_mode = record_mode
        self.recorder = SFRecorder(
            task_name,
            self.object_dict,
            self.robot_dict,
            self.mj_scene,
            record_mode=self.record_mode
        )
        self.assign_controller(self.create_controller())

    def create_scene(self) -> MjScene:
        self.object_dict = self.create_objects()
        return self.sim_factory.create_scene(
            object_list=self.object_dict.values(),
        )

    def reset_objects(self, obj_state: Dict):
        for obj_name, obj_state in obj_state.items():
            self.mj_scene.set_obj_pos_and_quat(
                obj_name=obj_name,
                new_pos=obj_state["pos"],
                new_quat=obj_state["quat"],
            )

    def reset_robots(self, robot_state: Dict):
        for robot_name, robot_state in robot_state.items():
            self.robot_dict[robot_name].beam_to_joint_pos(
                robot_state["joint_pos"]
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

    @abc.abstractmethod
    def reset(self):
        raise NotImplementedError

    def assign_controller(self, controller_dict: Dict[str, ControllerBase]):
        self.controller_dict = controller_dict
        for name, robot in self.robot_dict.items():
            self.controller_dict[name].executeController(
                robot, maxDuration=1000, block=False
            )

    def before_step(self):
        pass

    def after_step(self):
        pass

    def run(self):
        # count = 0
        self.reset()
        while True:
            self.before_step()
            self.mj_scene.next_step()
            if self.record_mode:
                self.recorder.record()
            self.after_step()
