from alr_sim.core.Scene import Scene
from alr_sim.core.sim_object import SimObject

from utils.unity_utils import mj2unity_size, mj2unity_pos, mj2unity_quat


class ObjectHandler:
    def __init__(self, obj: SimObject, scene: Scene) -> None:
        self.obj = obj
        self.scene = scene

        if hasattr(obj, "name"):
            self.name = obj.name
        elif hasattr(obj, "object_name"):
            self.name = getattr(obj, "object_name")
        else:
            raise Exception("Cannot find object name")

        if hasattr(obj, "type"):
            obj_type = getattr(obj, "type")
            if type(obj_type) is str:
                self.type = obj_type
            elif hasattr(obj_type, "value"):
                self.type = getattr(obj_type, "value")
        else:
            raise Exception("Cannot find object type")

    def get_obj_param_dict(self):
        obj = self.obj
        return {
            "attr": {
                "type": self.type,
            },
            "data": {
                "pos": list(mj2unity_pos(self.scene.get_obj_pos(obj))),
                "rot": list(mj2unity_quat(self.scene.get_obj_quat(obj))),
                "size": mj2unity_size(obj),
                "rgba": [-1, -1, -1, 1] if not hasattr(obj, "rgba") else obj.rgba,
                "rot_offset": [0, 0, 0]
                if not hasattr(obj, "rot_offset")
                else getattr(obj, "rot_offset"),
            },
        }

    def get_obj_state_dict(self) -> dict:
        return {
            "data": {
                "pos": list(mj2unity_pos(self.scene.get_obj_pos(self.obj))),
                "rot": list(mj2unity_quat(self.scene.get_obj_quat(self.obj))),
            }
        }