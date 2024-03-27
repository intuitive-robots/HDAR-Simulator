import numpy as np
from gym.spaces import Box as gym_box

from alr_sim.utils.geometric_transformation import euler2quat

from .TaskManager import TaskManager


class PracticalManipulationManager(TaskManager):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, task_type="practical_manipulation", **kwargs)

    def reset_objects(self):
        super().reset_objects()
        self.banana_space = gym_box(
            low=np.array([0.3, -0.1, -90]),
            high=np.array([0.35, 0.05, 90]),  # , seed=seed
        )
        self.mug_space = gym_box(
            low=np.array([0.4, 0.2, -90]), high=np.array([0.45, 0.3, 90])  # , seed=seed
        )
        self.context = self.sample()
        self.set_context(self.context)

    def set_context(self, context):
        red_pos, quat, green_pos, quat2 = context

        self.scene.set_obj_pos_and_quat(
            [red_pos[0], red_pos[1], 0.3],
            quat,
            obj_name="banana",
        )

        self.scene.set_obj_pos_and_quat(
            [green_pos[0], green_pos[1], 0.4],
            quat2,
            obj_name="mug",
        )

    def sample(self):
        banana_pos = self.banana_space.sample()
        mug_pos = self.mug_space.sample()

        goal_angle = [0, 0, banana_pos[-1] * np.pi / 180]
        quat = euler2quat(goal_angle)

        goal_angle2 = [0, 0, mug_pos[-1] * np.pi / 180]
        # quat2 = euler2quat(goal_angle2)
        quat2 = [0.0, 1.0, 0.0, 0.0]

        return [banana_pos, quat, mug_pos, quat2]
