import torch
import time
import numpy as np
from typing import List
from enum import Flag, auto

from alr_sim.core import Scene, RobotBase
from alr_sim.controllers.Controller import ControllerBase
from alr_sim.sims.SimFactory import SimRepository

import server, controllers, utils, tasks, poly_controllers


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
    # RESETTING = auto()
    RESETTASK0, RESETTASK1, RESETTASK2, RESETTASK3 = auto(), auto(), auto(), auto()
    RESETTING = RESETTASK0 | RESETTASK1 | RESETTASK2 | RESETTASK3

    CONNECTED = ON_TASK | WAITING_FOR_RESET | RESETTING
    RUNNING = WAITING_FOR_CONNECTION | CONNECTED


class Simulation:
    def __init__(
        self,
        task_type,
        record_mode=False,
        time_limit=1000000000,
        dt=0.002,
        downsample_steps=1,
    ) -> None:

        self.vt_factory = SimRepository.get_factory("mj_beta")

        self.time_limit = time_limit
        self.task_type = task_type
        self.record_mode = record_mode
        self.downsample_steps = downsample_steps
        self.dt = dt

        # virtual twin
        self.task_manager = tasks.get_manager(
            task_type, self.vt_factory, self.dt
        )
        self.task_manager.create_task()

        self.vt_scene: Scene = self.task_manager.get_scene()
        self.vt_object_list = self.task_manager.get_object_list()
        self.vt_robot_dict = self.task_manager.get_robot_dict()

        self.setup_controllers()
        self.start_scenes()
        self.start_controllers()
        self.setup_callbacks()

        self.setup_streamer()
        self.streamer.start()

        self.setup_recorder()

        self.setup_cli()
        self.cli.start()

        self.current_time = 0
        self.status: SimFlag = SimFlag.RUNNING

        self.force_interval = 0.02
        self.force_last_timestep = -self.force_interval

    def setup_callbacks(self):
        self.reset_flag = False
        self.start_record_flag = False
        self.stop_record_flag = False
        self.save_record_flag = False
        self.change_task_flag = None
        self.vt_scene.register_callback(self._reset_clb)
        self.vt_scene.register_callback(self._start_record_clb)
        self.vt_scene.register_callback(self._stop_record_clb)
        self.vt_scene.register_callback(self._save_record_clb)
        self.vt_scene.register_callback(self._change_task_clb)
        self.vt_scene.register_callback(self._max_time_clb)
        self.vt_scene.register_callback(self._task_finished_clb)

    def setup_cli(self):
        # GLI setting
        self.cli = utils.CLI()
        self.cli.register_function("Q", "close", self.shutdown_cli)
        self.cli.register_function("R", "Reset", self.reset)
        # self.cli.register_function("OG", "Open Gripper", self.open_grippers)
        # self.cli.register_function("CG", "Close Gripper", self.close_grippers)
        self.cli.register_function("D", "Start Record", self.start_record)
        # self.cli.register_function("SPR", "Stop Record", self.stop_record)
        self.cli.register_function("L", "Save and Reset", self.save_and_reset)
        self.cli.register_function("SQR", "Start QR Teleport", self.streamer.start_qr_teleport)
        self.cli.register_function("CQR", "Close QR Teleport", self.streamer.close_qr_teleport)
        self.cli.register_function(
            "MT", "Set Objects Manipulable True", self.streamer.set_objects_manipulable_true
        )
        self.cli.register_function(
            "MF", "Set Objects Manipulable False", self.streamer.set_objects_manipulable_false
        )
        self.cli.register_function(
            "AEER",
            "Activate End Effector Recorder",
            self.streamer.activate_end_effector_recorder,
        )
        self.cli.register_function(
            "DEER",
            "Deactivate End Effector Recorder",
            self.streamer.deactivate_end_effector_recorder,
        )
        self.cli.register_function("0", "Reset Task 0", self.change2task("warm_up"))
        self.cli.register_function(
            "1", "Reset Task 1", self.change2task("box_stacking")
        )
        self.cli.register_function(
            "2", "Reset Task 2", self.change2task("cup_stacking")
        )
        self.cli.register_function(
            "3", "Reset Task 3", self.change2task("practical_manipulation")
        )

    def setup_recorder(self):
        self.recorder = server.UnityRecorder(
            self.vt_scene,
            self.vt_object_list,
            self.task_type,
            self.streamer,
            manager=self.task_manager,
            record_mode=self.record_mode,
            downsample_steps=self.downsample_steps,
        )

    def setup_streamer(self):
        hdar_config = utils.get_hdar_config()
        self.streamer = server.UnityStreamer(
            self.vt_scene,
            self.vt_robot_dict.values(),
            self.vt_object_list,
            **hdar_config,
        )
        # self.streamer.register_callback(start_record=self.start_record)
        # self.streamer.register_callback(stop_record=self.stop_record)
        self.streamer.register_callback(reset=self.reset)
        self.streamer.register_callback(save_and_reset=self.save_and_reset)
        self.streamer.register_callback(open_grippers=self.open_grippers)
        self.streamer.register_callback(close_grippers=self.close_grippers)

    def setup_controllers(self):
        self.controller_list: List[ControllerBase] = list()
        self.robot_list: List[RobotBase] = list()
        self.real_robot_list: List[poly_controllers.Panda] = list()
        self.scene_list: List[Scene] = [self.vt_scene]

        for robot_name, robot_config in self.task_manager.robots_config.items():
            # for vt_robot_handler in self.streamer.robot_handlers:
            vt_robot = self.vt_robot_dict[robot_name]
            interaction_method = vt_robot.interaction_method
            real_robot = None 
            if interaction_method == "real_robot":
                # set up real scene
                real_robot_config = utils.get_real_robot_config()[robot_name]
                real_robot = poly_controllers.Panda(
                    name=robot_name,
                    **real_robot_config,
                )
                real_controller = controllers.RealRobotController(real_robot)
                self.real_robot_list.append(real_robot)

                vt_controller = controllers.VTController(
                    vt_robot,
                    real_robot,
                    self.vt_scene,
                    robot_config,
                )

            else:
                if interaction_method == "gamepad":
                    controller_cls = controllers.GamePadTCPController
                elif interaction_method == "virtual_robot":
                    controller_cls = controllers.VirtualRobotTCPController
                elif interaction_method == "hand_tracking":
                    controller_cls = controllers.HandTrackerTCPController
                elif interaction_method == "motion_controller":
                    controller_cls = controllers.ViveProMotionControllerTCPController
                vt_controller = controller_cls(
                    self.vt_scene,
                    vt_robot,
                    robot_config,
                )

            self.controller_list.append(vt_controller)
            self.robot_list.append(vt_robot)

    def start_scenes(self):
        for scene in self.scene_list:
            scene.start()

    def start_controllers(self):
        for robot, controller in zip(self.robot_list, self.controller_list):
            if isinstance(controller, controllers.VTController):
                robot.beam_to_joint_pos(
                    controller.real_robot.robot.get_joint_positions().numpy()
                )
            controller.executeController(robot, maxDuration=1000, block=False)
    
    def change2task(self, task_type):
        def f():
            self.change_task_flag = task_type

        return f

    def _change_task_clb(self):
        if self.change_task_flag is not None:
            task_type = self.change_task_flag
            self.change_task_flag = None
            if hasattr(self.task_manager, "change2task"):
                self.task_manager.change2task(task_type)

    def reset(self):
        self.stop_record_flag = True
        self.reset_flag = True

    def _reset_clb(self):
        if self.reset_flag:
            self.reset_flag = False
            self.reset_initial_pose()

    def start_record(self):
        self.start_record_flag = True

    def _start_record_clb(self):
        if self.start_record_flag:
            self.start_record_flag = False
            self.stop_record_flag = False
            self.recorder.start_record()

    def stop_record(self):
        self.stop_record_flag = True

    def _stop_record_clb(self):
        if self.stop_record_flag:
            self.stop_record_flag = False
            self.recorder.stop_record()

    def save_and_reset(self):
        self.save_record_flag = True
        self.reset_flag = True

    def _save_record_clb(self):
        if self.save_record_flag:
            self.save_record_flag = False
            self.recorder.save_record()

    def _max_time_clb(self):
        if self.recorder.on_logging:
            if self.current_time < self.time_limit:
                self.current_time += 1
            else:
                self.current_time = 0
                print("Time is up")
                self.stop_record_flag = True

    def _task_finished_clb(self):
        if self.task_manager.is_task_finished() and self.status is SimFlag.ON_TASK:
            self.stop_record_flag = True
            self.streamer.send_message("task_finished")
            self.status = SimFlag.WAITING_FOR_RESET

    def open_grippers(self, *args, robot_name="panda1", **kwargs):
        self.vt_robot_dict[robot_name].open_fingers()

    def close_grippers(self, *args, robot_name="panda1", **kwargs):
        self.vt_robot_dict[robot_name].close_fingers(duration=0)

    def terminate_policies(self):
        for real_robot in self.real_robot_list:
            if real_robot.is_running_policy():
                real_robot.robot.terminate_current_policy()

    def shutdown_cli(self, *args, **kwargs):
        self.recorder.stop_record()
        self.streamer.close_server()
        for controller in self.controller_list:
            controller._max_duration = 0
        self.terminate_policies()
        self.status = SimFlag.SHUTDOWN
        self.streamer.shutdown()
        return False

    def reset_initial_pose(self):
        self.status = SimFlag.RESETTING
        self.task_manager.reset_objects()
        self.task_manager.reset_robots()

        self.streamer.send_message("new_task_ready")
        self.status = SimFlag.ON_TASK

    def update_force_feedback(self):
        if (
            self.scene_list[0].time_stamp
            > self.force_last_timestep + self.force_interval
        ):
            self.force_last_timestep = self.scene_list[0].time_stamp
            for vt_robot, real_robot in zip(self.robot_list, self.real_robot_list):
                if real_robot.is_running_policy():
                    constraint_forces = [
                        self.vt_scene.data.joint(name).qfrc_constraint[0]
                        for name in vt_robot.joint_names
                    ]
                    constraint_forces = torch.tensor(np.array(constraint_forces))
                    try:
                        real_robot.update_constraint_forces(constraint_forces)
                    except:
                        pass

    def run(self):
        self.terminate_policies()
        self.reset_initial_pose()
        steps = 0
        start = time.time()
        while self.status in SimFlag.RUNNING:
            while steps * self.dt > time.time() - start:
                pass
            steps += 1
            self.update_force_feedback()
            for scene in self.scene_list:
                scene.next_step()
        print("Goodbye")