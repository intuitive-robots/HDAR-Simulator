import abc
import queue
from typing import List, Dict, Any
from typing import Callable, Tuple
import numpy as np
from alr_sim.core.Scene import Scene
from alr_sim.sims.mj_beta import MjRobot
from alr_sim.sims.mj_beta import MjScene
from alr_sim.controllers import ControllerBase
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import MujocoObject
from simpub.sim.sf_publisher import SFPublisher
import poly_controllers,controllers

# TaskQueue = queue.Queue[Tuple[Callable[..., None], Tuple[Any, ...]]]
TaskTuple = Tuple[Callable[..., None], Tuple[Any, ...]]
TaskQueue = queue.Queue  # Type hint only, no direct subscript


class SFSimulator(abc.ABC):

    def __init__(
        self,
        task_name: str,
        host_address: str = None,
    ):
        # NOTE: for all the possible simulator
        self.task_name = task_name
        # MJSimFactory
        self.sim_factory = SimRepository.get_factory("mj_beta")
        self.mj_scene = self.create_scene()
        self.robot_dict = self.create_robots()
        # self.task_queue: TaskQueue = TaskQueue()
        self.task_queue = queue.Queue()
        self.mj_scene.start()
        if host_address is not None:
            self.publisher = SFPublisher(self.mj_scene, host_address)

    def create_scene(
        self,
        dt=0.001,
        render=Scene.RenderMode.HUMAN,
        surrounding=None,
    ) -> MjScene:
        self.object_dict = self.create_objects()
        return self.sim_factory.create_scene(
            object_list=self.object_dict.values(),
            dt=dt,
            render=render,
            surrounding=surrounding,
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

    def add_task(self, func: Callable[..., None], *args: Any) -> None:
        self.task_queue.put((func, args))

    def reset_in_the_main_thread(self):
        self.task_queue.put((self.reset, None))

    @abc.abstractmethod
    def create_robots(self) -> Dict[str, MjRobot]:
        raise NotImplementedError

    @abc.abstractmethod
    def create_objects(self) -> Dict[str, MujocoObject]:
        raise NotImplementedError

    @abc.abstractmethod
    def reset(self):
        raise NotImplementedError
    

    @abc.abstractmethod
    def before_step(self):
        raise NotImplementedError

    @abc.abstractmethod
    def after_step(self):
        raise NotImplementedError

    def assign_controller(self, controller_dict: Dict[str, ControllerBase]):
        self.controller_dict = controller_dict
        
        print(f"controller dict:{controller_dict}")
        
        for name, robot in self.robot_dict.items():
            controller = controller_dict.get(name)
            
            if isinstance(controller, controllers.vt_controller.VTController):
                pos=controller.real_robot.robot.get_joint_positions()
                robot.beam_to_joint_pos( pos.numpy()
                    # controller.real_robot.robot.get_joint_positions().numpy()
                )
                print(f"finish to set controller pos")
            self.controller_dict[name].executeController(
                robot, maxDuration=1000, block=False
            )

    def execute_task_queue(self):
        while not self.task_queue.empty():
            task, args = self.task_queue.get(block=False)
            if args is None:
                task()
            else:
                task(*args)

    def next_step(self):
        self.before_step()
        self.mj_scene.next_step()
        self.execute_task_queue()
        self.after_step()

    def run(self):
        self.reset()
        while True:
            self.next_step()

    def reset_controlled_robots(self,target_pos=None):
        for robot, target_pos in zip(self.robot_dict.values(), target_pos):
        # for robot in self.robot_dict.values():
            if hasattr(robot.activeController, "reset_robot"):
                robot.activeController.reset_robot(target_pos=target_pos)
    
    def reset_real_robot_in_the_main_thread(self):
        self.task_queue.put((self.reset_real_robot, None))

    def reset_real_robot(self):
        target_pos=self.reset()
        self.reset_controlled_robots(target_pos=target_pos)