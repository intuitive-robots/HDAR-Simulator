import threading
import signal
import spdlog
import time

from tqdm import tqdm
from queue import Queue
from typing import Optional, List

from polymetis_pb2 import RobotState
from palrymetis import Panda

ROBOT_STATE_MEMBERS = [
    "joint_positions",
    "joint_torques_computed",
    "joint_velocities",
    "motor_torques_desired",
    "motor_torques_external",
    "motor_torques_measured",
    "prev_command_successful",
    "prev_controller_latency_ms",
    "prev_joint_torques_computed",
    "prev_joint_torques_computed_safened",
    "timestamp",
]

GRIPPER_STATE_MEMBERS = ["is_grasped", "is_moving", "width"]


class Recorder:
    def __init__(
        self,
        panda: Panda,
        running: callable,
        subscriptions: Optional[List[str]] = None,
        hz: float = None,
    ):
        """ """
        self.logger = spdlog.ConsoleLogger("recorder")
        # self.logger.set_level(spdlog.LogLevel.DEBUG)
        self.panda = panda
        self.recordings = {}
        self.running = running
        self.record = False
        self.hz = hz if hz else self.panda.robot.hz
        self.period = 1 / self.hz

        subscriptions = (
            subscriptions
            if subscriptions
            else ROBOT_STATE_MEMBERS + GRIPPER_STATE_MEMBERS
        )

        for sub in subscriptions:
            if (
                sub not in ROBOT_STATE_MEMBERS + GRIPPER_STATE_MEMBERS
            ):  # and not sub in GRIPPER_STATE_MEMBERS
                self.logger.error(
                    f"No subscriptable member called {sub} found.\nTry {ROBOT_STATE_MEMBERS}"
                )  # or {GRIPPER_STATE_MEMBERS}")

        self.subscriptions = subscriptions

        for sub in self.subscriptions:
            self.recordings[sub] = []

        self.i = 0
        self.t0 = time.time()
        self.last_timestamp = 0

        self.logger.debug("Starting record thread")
        self.thread = threading.Thread(target=self._record)
        self.thread.start()

    def _record(self):
        self.last_timestamp = 0
        while self.running():
            if self.record:
                state = self.panda.get_state()

                # skip if we receive the same state
                if not self.last_timestamp == state["robot"].timestamp:
                    for sub in self.subscriptions:
                        if sub in GRIPPER_STATE_MEMBERS:
                            self.recordings[sub].append(getattr(state["gripper"], sub))
                        else:
                            self.recordings[sub].append(getattr(state["robot"], sub))
                    self.i += 1
                    delta = self.t0 + self.period * self.i - time.time()
                    if delta > 0:
                        time.sleep(delta)

                self.last_timestamp = state["robot"].timestamp

    def start(self):
        self.logger.info("Started recording")
        self.record = True

    def stop(self):
        self.logger.info("Stopped recording")
        self.record = False

    def cleanup(self):
        """ """
        self.thread.join()

    def save(self):
        """ """
        import os
        import pandas
        from datetime import datetime

        save_dir = "outputs/recordings/"

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        filename = f"{timestamp}.csv"
        full_path = os.path.join(save_dir, filename)

        self.logger.info(f"Saving to {full_path}")
        if "timestamp" in self.subscriptions:
            self.recordings["timestamp"] = [
                ts.seconds * 1e9 + ts.nanos for ts in self.recordings["timestamp"]
            ]
        pandas.DataFrame(self.recordings).to_csv(
            full_path, float_format="%32.32f", index=False
        )
