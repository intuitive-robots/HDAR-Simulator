import argparse
from omni.isaac.orbit.app import AppLauncher

parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import utils

import omni.isaac.orbit.sim as sim_utils


import time
from typing import List
from enum import Flag, auto

# import server, controllers, utils, tasks, poly_server
import tasks, poly_server

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.controllers.joint_impedance import JointImpedanceController, JointImpedanceControllerCfg


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
        sim_cfg = sim_utils.SimulationCfg(dt=dt, substeps=downsample_steps)
        self.sim = sim_utils.SimulationContext(sim_cfg)
        self.sim.set_camera_view([2.5, 0.34, 1.5], [0.0, 0.34, 0.5])

        self.time_limit = time_limit
        self.task_type = task_type
        self.record_mode = record_mode
        self.downsample_steps = downsample_steps
        self.dt = dt

        # virtual twin
        self.task_manager = tasks.get_manager(
            task_type, self.dt
        )
        self.task_manager.create_task()
        self.sim.reset()

        # self.vt_scene: Scene = self.task_manager.get_scene()
        # self.vt_object_list = self.task_manager.get_object_list()
        self.vt_robot_dict = self.task_manager.get_robot_dict()

        self.setup_controllers()
        # self.start_scenes()
        # self.start_controllers()
        # self.setup_callbacks()

        # self.setup_streamer()
        # self.streamer.start()

        # self.setup_recorder()

        # self.setup_cli()
        # self.cli.start()

        self.poly_streamer = poly_server.PolyServer(self.controller_list)
        self.poly_streamer.start()

        # self.current_time = 0
        self.status: SimFlag = SimFlag.RUNNING

        # self.force_interval = 0.02
        # self.force_last_timestep = -self.force_interval

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
        self.controller_list = {}
        self.robot_list: List = list()
        # self.scene_list: List = [self.vt_scene]


        for robot_name, robot_config in self.task_manager.robots_config.items():
            # for vt_robot_handler in self.streamer.robot_handlers:
            vt_robot = self.vt_robot_dict[robot_name]
            interaction_method = vt_robot.interaction_method
            assert interaction_method == 'real_robot'

            joint_stiffness = vt_robot.data.joint_stiffness.clone()
            controller_cfg = JointImpedanceControllerCfg(
                impedance_mode='fixed',
                stiffness=joint_stiffness,
                damping_ratio=1.)
            joint_limits = vt_robot.data.soft_joint_pos_limits.clone()
            controller = JointImpedanceController(
                controller_cfg,
                num_robots=1,
                dof_pos_limits=joint_limits,
                device=self.sim.device)
            # if interaction_method == "real_robot":
            #     controller_cls = controllers.VTController
            # elif interaction_method == "gamepad":
            #     controller_cls = controllers.GamePadTCPController
            # elif interaction_method == "virtual_robot":
            #     controller_cls = controllers.VirtualRobotTCPController
            # elif interaction_method == "hand_tracking":
            #     controller_cls = controllers.HandTrackerTCPController
            # elif interaction_method == "motion_controller":
            #     controller_cls = controllers.ViveProMotionControllerTCPController
            # vt_controller = controller_cls(
            #     self.vt_scene,
            #     vt_robot,
            #     robot_config,
            # )

            self.controller_list[robot_name] = controller
            # self.robot_list.append(vt_robot)

    def start_scenes(self):
        for scene in self.scene_list:
            scene.start()

    def start_controllers(self):
        for robot, controller in zip(self.robot_list, self.controller_list.values()):
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

    def shutdown_cli(self, *args, **kwargs):
        self.recorder.stop_record()
        self.streamer.close_server()
        for controller in self.controller_list.values():
            controller._max_duration = 0
        self.status = SimFlag.SHUTDOWN
        self.streamer.shutdown()
        self.poly_streamer.shutdown()
        return False

    def reset_initial_pose(self):
        self.status = SimFlag.RESETTING
        self.sim.reset(soft=True)
        # self.task_manager.reset_objects()
        # self.task_manager.reset_robots()

        # self.streamer.send_message("new_task_ready")
        self.status = SimFlag.ON_TASK

    def run(self):
        # self.reset_initial_pose()
        steps = 0
        start = time.time()
        while simulation_app.is_running():
            # while steps * self.dt > time.time() - start:
                # pass
            # steps += 1
            for robot_name, controller in self.controller_list.items():
                robot = self.vt_robot_dict[robot_name]
                joint_pos = robot.data.joint_pos.clone()
                joint_vel = robot.data.joint_vel.clone()
                joint_pos_des = controller.compute(joint_pos, joint_vel)
            
                robot.set_joint_position_target(joint_pos_des)

            self.sim.step()
        print("Goodbye")


if __name__ == "__main__":
    simulator_config = utils.get_simulator_config()
    Simulation(**simulator_config).run()
    simulation_app.close()