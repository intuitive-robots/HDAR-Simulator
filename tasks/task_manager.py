import numpy as np
from typing import List, Dict
import yaml

# from alr_sim.core import RobotBase, Scene
# from alr_sim.sims.SimFactory import SimFactory
# from alr_sim.core.sim_object import SimObject
# from alr_sim.utils import sim_path
# from alr_sim.utils.geometric_transformation import euler2quat

import utils #, controllers
# from server.hdar_model import generate_HDARObj_from_dict


import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit_assets import FRANKA_PANDA_HIGH_PD_CFG
from omni.isaac.orbit.assets import Articulation, ArticulationCfg


def euler2quat(euler):
    """Convert Euler Angles to Quaternions.  See rotation.py for notes"""
    euler = np.asarray(euler, dtype=np.float64)
    assert euler.shape[-1] == 3, "Invalid shape euler {}".format(euler)

    ai, aj, ak = euler[..., 2] / 2, -euler[..., 1] / 2, euler[..., 0] / 2
    si, sj, sk = np.sin(ai), np.sin(aj), np.sin(ak)
    ci, cj, ck = np.cos(ai), np.cos(aj), np.cos(ak)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    quat = np.empty(euler.shape[:-1] + (4,), dtype=np.float64)
    quat[..., 0] = cj * cc + sj * ss
    quat[..., 3] = cj * sc - sj * cs
    quat[..., 2] = -(cj * ss + sj * cc)
    quat[..., 1] = cj * cs - sj * sc
    return quat


_repository = {}

def get_task_setting(task_type: str):
    task_setting_path = f"tasks/task_setting/{task_type}.yaml"
    with open(task_setting_path, "r") as f:
        task_setting = yaml.load(f, Loader=yaml.FullLoader)
    return task_setting


def get_manager(task_type: str, dt: float):
    if task_type in _repository:
        return _repository[task_type](dt=dt)
    print("Using default task manager")
    return TaskManager(task_type=task_type, dt=dt)


class TaskManager:
    def __init__(self, task_type: str, dt: float) -> None:
        self.object_dict = {}
        self.robot_dict = {}
        self.dt = dt

        task_setting = get_task_setting(task_type)
        self.objects_config: Dict[str, dict] = task_setting["objects"]
        self.robots_config: Dict[str, dict] = task_setting["robots"]
        robots_interaction_config = utils.get_virtual_robot_config()
        for robot_name, config in self.robots_config.items():
            config.update(robots_interaction_config[robot_name])

    def create_task(self):
        cfg_ground = sim_utils.GroundPlaneCfg()
        cfg_ground.func("/World/defaultGroundPlane", cfg_ground)
        cfg_light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        cfg_light.func("/World/Light", cfg_light)
        self.create_objects()
        self.create_robots()

    def get_object_dict(self):
        return self.object_dict

    def get_robot_dict(self):
        return self.robot_dict

    def get_object_list(self):
        return list(self.object_dict.values())

    def get_robot_list(self):
        return list(self.robot_dict.values())

    def get_scene(self):
        return self.scene

    def is_task_finished(self):
        return False

    def create_objects(self):
        for obj_name, obj_config in self.objects_config.items():
            if obj_config["source"] != "primitive_object":
                continue
            if obj_config["type"] != "Box":
                continue
            cfg_cuboid = sim_utils.CuboidCfg(
                size=obj_config["size"],
                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                mass_props=sim_utils.MassPropertiesCfg(mass=obj_config.get("mass", 1.0)),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(map(float, obj_config["rgba"][:3]))),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                # static=obj_config.get("static", False),
                # visual_only=obj_config.get("visual_only", False),
            )
            cfg_cuboid.func(
                f"/World/{obj_name}",
                cfg_cuboid,
                translation=obj_config['init_pos'], 
                orientation=obj_config['init_quat'],
            )
            
            self.object_dict[obj_name] = cfg_cuboid
    
    def _create_robot(self, name, config):
        robot_cfg = FRANKA_PANDA_HIGH_PD_CFG
        robot_cfg = robot_cfg.replace(prim_path=f"/World/{name}")
        base_position = config["base_position"]
        base_quat = euler2quat(
            np.array(config["base_rotation"]) * np.pi / 180
        )
        robot_cfg.init_state = robot_cfg.init_state.replace(
            pos=base_position,
            rot=base_quat
        )
        robot = Articulation(cfg=robot_cfg)
        model_name = config.get("type", "gripper_panda")
        # robot = self.sim_factory.create_robot(
        #     self.scene,
        #     num_DoF=7,
        #     base_position=base_position,
        #     base_orientation=base_quat,
        #     gravity_comp=True,
        #     clip_actions=False,
        #     root=sim_path.FRAMEWORK_DIR,
        #     xml_path=f"./model/robot/{model_name}.xml",
        # )
        robot.type = model_name
        robot.name = name
        # robot.init_end_eff_pos = config["init_end_eff_pos"]
        # robot.init_end_eff_quat = config["init_end_eff_quat"]
        robot.interaction_method = config["interaction_method"]
        return robot

    def create_robots(self):
        for robot_name, robot_config in self.robots_config.items():
            self.robot_dict[robot_name] = self._create_robot(robot_name, robot_config)

    def reset_objects(self):
        for obj_name, obj in self.object_dict.items():
            self.scene.set_obj_pos_and_quat(
                obj.init_pos, obj.init_quat, obj_name=obj_name
            )

    def reset_robots(self):
        for robot in self.get_robot_list():
            if hasattr(robot.activeController, "reset_robot"):
                robot.activeController.reset_robot()

    def _add_robot_attr():
        pass
