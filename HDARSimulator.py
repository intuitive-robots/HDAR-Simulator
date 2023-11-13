import time
from enum import Flag, auto, Enum
import numpy as np

# from alr_sim.controllers.Controller import ControllerBase
from alr_sim.core import Scene, RobotBase
from alr_sim.core.sim_object import SimObject
from alr_sim.controllers.Controller import ControllerBase
from alr_sim.sims.SimFactory import SimRepository


# from alr_sim.sims.sl.multibot_teleop.src.teleop_controller import TeleopController
from alr_sim.sims.sl.multibot_teleop.src.ui.cli import GeneralCLI
from alr_sim.sims.sl.SlRobot import SlRobot
from alr_sim.sims.sl.SlScene import SlScene
from alr_sim.sims.sl.SlFactory import SlFactory

from hdar_server.UnityHDRecorder import UnityHDRecorder
from hdar_server.UnityStreamer import UnityStreamer
from hdar_server.HDARController import *
from hdar_server.utils import get_hdar_config, str2list
from hdar_task.TaskManager import TaskManager


class HDARCLI(GeneralCLI):
    def get_input(self) -> bool:
        cmd = str(input("Enter Command... ")).upper()
        return self.exec_cmd(cmd)


class SimFlag(Flag):
    """
    - shutdown
    - running
        - waiting for server
        - connected
            - on stream
            - wait for reset
            - resetting
    """

    SHUTDOWN = auto()
    WAITING_FOR_CONNECTION = auto()
    ON_TASK = auto()
    WAITING_FOR_RESET = auto()
    RESETTING = auto()

    CONNECTED = ON_TASK | WAITING_FOR_RESET | RESETTING
    RUNNING = WAITING_FOR_CONNECTION | CONNECTED


class HDARSimulator:
    def __init__(
        self,
        task_type,
        record_mode=False,
        time_limit=999999,
        downsample_steps=1,
    ) -> None:

        self.real_sim_factory: SlFactory = None
        self.real_scene: SlScene = None

        self.current_time = 0
        self.time_limit = time_limit

        # virtual twin
        self.vt_sim_factory = SimRepository.get_factory("mj_beta")
        self.scene_manager: TaskManager = TaskManager.get_manager(task_type)
        self.scene_manager.create_task(self.vt_sim_factory)

        self.vt_scene: Scene = self.scene_manager.get_scene()
        self.vt_object_dict = self.scene_manager.get_object_dict()
        self.vt_robot_dict = self.scene_manager.get_robot_dict()

        self.robot_list: list[RobotBase] = list()

        self.scene_list: list[Scene] = list()
        self.scene_list.append(self.vt_scene)

        hdar_config = get_hdar_config("HDARConfig")
        self.streamer = UnityStreamer(
            self.vt_scene,
            self.vt_robot_dict.values(),
            self.vt_object_dict.values(),
            host=hdar_config["hdar_addr"],
            port=hdar_config["hdar_port"],
        )

        # controller setting
        self.vt_controller_dict: dict[
            str, VTController | InteractiveTCPControllerBase
        ] = {}
        self.controller_list: list[ControllerBase] = list()

        for robot_name, robot_config in self.scene_manager.robots_config.items():
            # for vt_robot_handler in self.streamer.robot_handlers:
            vt_robot = self.vt_robot_dict[robot_name]
            interaction_method = getattr(vt_robot, "interaction_method")
            if interaction_method == "real_robot":
                # set up real scene
                if self.real_sim_factory is None:
                    self.real_sim_factory = SimRepository.get_factory("sl")
                    self.real_scene = self.real_sim_factory.create_scene(skip_home=True)
                    self.scene_list.append(self.real_scene)
                sl_config = get_hdar_config("SLConfig")[robot_name]
                real_robot: SlRobot = self.real_sim_factory.create_robot(
                    self.real_scene,
                    robot_name=sl_config["sl_robot_name"],
                    backend_addr=sl_config["backend_addr"],
                    local_addr=sl_config["local_addr"],
                    gripper_actuation=True,
                )
                real_controller = RealRobotController(real_robot, regularize=True)
                self.controller_list.append(real_controller)
                self.robot_list.append(real_robot)

                vt_controller = VTController(
                    real_robot,
                    self.vt_scene,
                    lambda: real_robot.current_j_pos,
                    lambda: real_robot.current_j_vel,
                    robot_config,
                    lambda: real_robot.gripper_width,
                )

            elif interaction_method == "gamepad":
                vt_controller = GamePadTCPController(
                    self.vt_scene,
                    vt_robot,
                    robot_config,
                )
            elif interaction_method == "virtual_robot":
                vt_controller = VirtualRobotTCPController(
                    self.vt_scene,
                    vt_robot,
                    robot_config,
                )
            elif interaction_method == "hand_tracking":
                vt_controller = HandTrackerTCPController(
                    self.vt_scene,
                    vt_robot,
                    robot_config,
                )
            elif interaction_method == "motion_controller":
                vt_controller = ViveProMotionControllerTCPController(
                    self.vt_scene,
                    vt_robot,
                    robot_config,
                )

            self.vt_controller_dict[robot_name] = vt_controller
            self.controller_list.append(vt_controller)
            self.robot_list.append(vt_robot)

        for scene in self.scene_list:
            scene.start()

        for robot, controller in zip(self.robot_list, self.controller_list):
            controller.executeController(robot, maxDuration=1000, block=False)

        # the recorder for logging and saving
        self.recorder: UnityHDRecorder = UnityHDRecorder(
            self.vt_scene,
            self.vt_object_dict.values(),
            task_type,
            self.streamer,
            manager=self.scene_manager,
            record_mode=record_mode,
            downsample_steps=downsample_steps,
        )

        # self.start_streamer()

        # GLI setting
        self.cli = HDARCLI()
        self.cli.register_function("Q", "close", self.shutdown_cli)
        self.cli.register_function("R", "Reset", self.reset)
        # self.cli.register_function("A", "Open Gripper", self.open_grippers)
        # self.cli.register_function("D", "Close Gripper", self.close_grippers)
        self.cli.register_function("D", "Start Record", self.start_record)
        # self.cli.register_function("SPR", "Stop Record", self.stop_record)
        self.cli.register_function("L", "Save and Reset", self.save_and_reset)
        self.cli.register_function("SQR", "Start QR Teleport", self.start_qr_teleport)
        self.cli.register_function("CQR", "Close QR Teleport", self.close_qr_teleport)
        self.cli.register_function(
            "MT", "Set Objects Manipulable True", self.set_objects_manipulable_true
        )
        self.cli.register_function(
            "MF", "Set Objects Manipulable False", self.set_objects_manipulable_false
        )
        self.cli.register_function(
            "AEER",
            "Activate End Effector Recorder",
            self.activate_end_effector_recorder,
        )
        self.cli.register_function(
            "DEER",
            "Deactivate End Effector Recorder",
            self.deactivate_end_effector_recorder,
        )
        self.cli.start()

        self.status: SimFlag = SimFlag.RUNNING

    def start_streamer(self):
        self.streamer.register_callback(start_record=self.start_record)
        self.streamer.register_callback(stop_record=self.stop_record)
        self.streamer.register_callback(reset=self.reset)
        self.streamer.register_callback(save_and_reset=self.save_and_reset)
        self.streamer.register_callback(open_grippers=self.open_grippers)
        self.streamer.register_callback(close_grippers=self.close_grippers)

        self.streamer.start()

    def start_qr_teleport(self):
        qr_config = get_hdar_config("QRConfig")
        self.streamer.send_dict(
            {
                "Header": "text_message",
                "TextMessage": "start_qr_teleport",
                "Data": {
                    "QRConfig": {
                        "attr": {"QRText": qr_config["QRText"]},
                        "data": {
                            "offsetPos": qr_config["offsetPos"],
                            "offsetRot": qr_config["offsetRot"],
                        },
                    },
                },
            }
        )

    def close_qr_teleport(self):
        self.streamer.send_message("close_qr_teleport")

    def set_objects_manipulable_true(self):
        self.streamer.send_message("set_objects_manipulable_true")

    def set_objects_manipulable_false(self):
        self.streamer.send_message("set_objects_manipulable_false")

    def activate_end_effector_recorder(self):
        self.streamer.send_message("activate_end_effector_recorder")

    def deactivate_end_effector_recorder(self):
        self.streamer.send_message("deactivate_end_effector_recorder")

    def start_record(self, *args, **kwargs):
        self.current_time = time.time()
        self.recorder.start_record()

    def stop_record(self, *args, **kwargs):
        self.recorder.stop_record()

    def reset(self, *args, **kwargs):
        self.recorder.stop_record()
        self.status = SimFlag.RESETTING

    def save_and_reset(self, *args, **kwargs):
        self.recorder.save_record()
        self.status = SimFlag.RESETTING

    # TODO: open close fingers by robot id
    def open_grippers(self, *args, **kwargs):
        self.vt_robot_dict["grasp_robot"].open_fingers()

    def close_grippers(self, *args, **kwargs):
        self.vt_robot_dict["grasp_robot"].close_fingers(duration=0)

    def shutdown_cli(self, *args, **kwargs):
        self.recorder.stop_record()
        self.streamer.close_server()
        for controller in self.controller_list:
            controller._max_duration = 0
        self.status = SimFlag.SHUTDOWN
        self.streamer.shutdown()
        return False

    def reset_initial_pose(self):
        # print("Recovering initial pose")
        self.scene_manager.reset_objects()
        for controller in self.vt_controller_dict.values():
            controller.reset_robot()

        self.streamer.send_message("new_task_ready")
        self.status = SimFlag.ON_TASK

        # just for flying object bug test
        # for obj in self.streamer.obj_list:
        #     self.data[obj.name]["pos"] = self.vt_scene.get_obj_pos(obj)

    def run(self):
        cam = self.vt_scene.viewer.cam
        cam.azimuth = 135
        cam.elevation = -25
        cam.distance = 2

        self.reset_initial_pose()
        # last_time = time.time()
        while self.status in SimFlag.RUNNING:
            # check whether current is finished
            if self.scene_manager.is_task_finished() and self.status is SimFlag.ON_TASK:
                self.recorder.stop_record()
                self.streamer.send_message("task_finished")
                self.status = SimFlag.WAITING_FOR_RESET
            # check the resetting status

            if self.status is SimFlag.RESETTING:
                self.reset_initial_pose()

            for scene in self.scene_list:
                scene.next_step()

            if (
                self.recorder.on_logging
                and time.time() - self.current_time >= self.time_limit
            ):
                print("Time is up")
                self.stop_record()

        # Shutdown Procedure
        print("Goodbye")


if __name__ == "__main__":
    # s = HDARSimulator(debug_mode=True)
    s = HDARSimulator(
        # task_type="grasp_object",
        # task_type="bimanual_test",
        # task_type="stacking",
        # task_type="cube_stacking",
        # task_type="warm_up",
        # task_type="box_stacking",
        # task_type="cup_stacking",
        task_type="practical_manipulation",
        record_mode=True,
        time_limit=60,
        downsample_steps=50,
    )
    s.run()
