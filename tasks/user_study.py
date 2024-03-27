from .task_manager import TaskManager, get_task_setting


class UserStudyManager(TaskManager):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, task_type="user_study", **kwargs)

    def reset_objects(self):
        i = 5
        for obj_name, obj in self.object_dict.items():
            init_pose = [obj.init_pos[0] + i, obj.init_pos[1], obj.init_pos[2] - 1.2]
            self.scene.set_obj_pos_and_quat(init_pose, obj.init_quat, obj_name=obj_name)
            i += 0.2

    def change2task(self, task_type):
        self.reset_objects()
        task_setting = get_task_setting(task_type)
        objects_config: dict[str, dict] = task_setting["objects"]
        for obj_name in objects_config.keys():
            obj = self.object_dict[obj_name]
            self.scene.set_obj_pos_and_quat(
                obj.init_pos, obj.init_quat, obj_name=obj_name
            )

    def is_task_finished(self):
        return True
        return super().is_task_finished()
