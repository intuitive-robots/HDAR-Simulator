from .HDARTaskBase import *


class CubeStackingManager(TaskManager):
    def __init__(self) -> None:
        super().__init__("cube_stacking")

        np.random.seed(42)
        self.red_space = GymBox(
            low=np.array([0.35, -0.25, -90]),
            high=np.array([0.45, -0.15, 90]),  # , seed=seed
        )

        self.green_space = GymBox(
            low=np.array([0.35, -0.1, -90]), high=np.array([0.45, 0, 90])  # , seed=seed
        )

        self.blue_space = GymBox(
            low=np.array([0.55, -0.2, -90]), high=np.array([0.6, 0, 90])  # , seed=seed
        )

        self.target_space = GymBox(
            low=np.array([0.4, 0.15, -90]),
            high=np.array([0.6, 0.25, 90]),  # , seed=seed
        )

    def create_objects(self):
        object_name = "red_box"
        object_config = self.objects_config[object_name]
        red_box = Box(
            name=object_name,
            init_pos=object_config["init_pos"],
            init_quat=object_config["init_quat"],
            rgba=object_config["rgba"],
            mass=object_config["mass"],
            size=object_config["size"],
        )
        self.object_dict[object_name] = red_box

        object_name = "green_box"
        object_config = self.objects_config[object_name]
        green_box = Box(
            name=object_name,
            init_pos=object_config["init_pos"],
            init_quat=object_config["init_quat"],
            rgba=object_config["rgba"],
            mass=object_config["mass"],
            size=object_config["size"],
        )
        self.object_dict[object_name] = green_box

        object_name = "blue_box"
        object_config = self.objects_config[object_name]
        blue_box = Box(
            name=object_name,
            init_pos=object_config["init_pos"],
            init_quat=object_config["init_quat"],
            rgba=object_config["rgba"],
            mass=object_config["mass"],
            size=object_config["size"],
        )
        self.object_dict[object_name] = blue_box

        object_name = "target_box"
        object_config = self.objects_config[object_name]
        target_box = Box(
            name=object_name,
            init_pos=object_config["init_pos"],
            init_quat=object_config["init_quat"],
            rgba=object_config["rgba"],
            size=object_config["size"],
            visual_only=True,
            static=True,
        )
        self.object_dict[object_name] = target_box

    def reset_task(self, random=True, context=None):
        for robot in self.robot_dict.values():
            robot.open_fingers()

        if random:
            self.context = self.sample()
        else:
            self.context = context

        self.set_context(self.context)

    def reset_objects(self):
        return self.reset_task()

    def is_task_finished(self):
        # return (
        #     objects_xy_distance("target_box", "blue_box", self.scene) < 0.01
        #     and objects_xy_distance("target_box", "blue_box", self.scene) < 0.01
        # )
        return False

    def sample(self):

        pos_1 = self.red_space.sample()
        angle_1 = [0, 0, pos_1[-1] * np.pi / 180]
        quat_1 = euler2quat(angle_1)

        pos_2 = self.green_space.sample()
        angle_2 = [0, 0, pos_2[-1] * np.pi / 180]
        quat_2 = euler2quat(angle_2)

        pos_3 = self.blue_space.sample()
        angle_3 = [0, 0, pos_3[-1] * np.pi / 180]
        quat_3 = euler2quat(angle_3)

        pos_4 = self.target_space.sample()
        angle_4 = [0, 0, pos_4[-1] * np.pi / 180]
        quat_4 = euler2quat(angle_4)

        return [pos_1, quat_1], [pos_2, quat_2], [pos_3, quat_3], [pos_4, quat_4]

    def set_context(self, context):

        red_pos = context[0][0]
        red_quat = context[0][1]

        green_pos = context[1][0]
        green_quat = context[1][1]

        blue_pos = context[2][0]
        blue_quat = context[2][1]

        self.scene.set_obj_pos_and_quat(
            [red_pos[0], red_pos[1], 0],
            red_quat,
            obj_name="red_box",
        )

        self.scene.set_obj_pos_and_quat(
            [green_pos[0], green_pos[1], 0],
            green_quat,
            obj_name="green_box",
        )

        self.scene.set_obj_pos_and_quat(
            [blue_pos[0], blue_pos[1], 0],
            blue_quat,
            obj_name="blue_box",
        )

        # target_pos = context[3][0]
        # target_quat = context[3][1]
        # self.scene.set_obj_pos_and_quat(
        #     [target_pos[0], target_pos[1], 0],
        #     [0, 1, 0, 0],
        #     obj_name="target_box",
        # )
