from alr_sim.sims.SimFactory import SimFactory
from .TaskManager import *
from alr_sim.utils.sim_path import sim_framework_path
from hdar_server.HDARModel import HDARSimObject
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import YCBMujocoObject


class BoxStackingManager(TaskManager):
    def __init__(self) -> None:
        super().__init__("box_stacking")

    def create_objects(self):
        super().create_objects()

    def create_robots(self, sim_factory: SimFactory):
        super().create_robots(sim_factory)

    def reset_objects(self):
        super().reset_objects()

    def is_task_finished(self):
        False
