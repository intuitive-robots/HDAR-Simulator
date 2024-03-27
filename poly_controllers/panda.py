from polymetis import RobotInterface, GripperInterface
from polymetis_pb2 import RobotState
from typing import List, Dict

import torchcontrol as toco
import spdlog
import time
import yaml


class Panda:
    def __init__(self, name, ip, robot_port, gripper_port):
        self.name = name
        self.logger = spdlog.ConsoleLogger(name)
        self.logger.set_level(spdlog.LogLevel.DEBUG)
        self.ip = ip
        self.robot_port = robot_port
        self.gripper_port = gripper_port
        self.last_policy = None

        self.connect()

    @classmethod
    def from_alr_name(cls, name):
        # TODO: config path (maybe env var?)
        panda_config = yaml.load(open("configs/robots.yaml"), yaml.FullLoader)[name]

        ip = "141.3.53.152"  # TODO: localhost for now, ip of rt PC otherwise

        panda_robot_port = panda_config["robot_port"]
        panda_gripper_port = panda_config["gripper_port"]

        return cls(name, ip, panda_robot_port, panda_gripper_port)

    def load_policy(
        self,
        policy: toco.PolicyModule = None,
        blocking: bool = True,
        timeout: float = None,
        use_mirror: bool = False,
    ) -> List[RobotState]:
        if policy == None:
            if self.last_policy:
                policy = self.last_policy
            else:
                return None

        if not self.is_connected():
            self.reconnect()

        if self.is_running_policy():
            self.logger.warn("Overwriting policy")

        self.last_policy = policy
        self.logger.debug(f"Loading {policy.__class__}")
        return self.robot.send_torch_policy(policy, blocking, timeout, use_mirror)

    def is_running_policy(self) -> bool:
        if self.robot:
            return self.robot.is_running_policy()
        return False

    def get_state(self) -> Dict:
        try:
            robot_state = self.robot.get_robot_state()
            gripper_state = self.gripper.get_state()
        except Exception as e:
            self.logger.warn(f"Failed to get current state: {e}")
            self.reconnect()
            return self.get_state()
        return {"robot": robot_state, "gripper": gripper_state}

    def connect(self) -> bool:
        try:
            self.logger.info(
                f"Connecting to Panda on {self.ip}:[robot_port: {self.robot_port}, gripper_port: {self.gripper_port}]"
            )
            self.robot = RobotInterface(
                ip_address=self.ip,
                port=self.robot_port,
                enforce_version=False,
            )
            self.gripper = GripperInterface(ip_address=self.ip, port=self.gripper_port)
            self.load_policy(blocking=False)
        except Exception as e:
            self.robot = None
            self.gripper = None
            self.logger.error(f"Failed with exception: {e}")

        return self.is_connected()

    def is_connected(self) -> bool:
        connected = False
        if self.robot and self.gripper:
            try:
                start = time.time()
                state = self.robot.get_robot_state()
                delay = time.time() - start
                assert delay < 10, "Acquired state is stale by {} seconds".format(delay)
                connected = True
            except:
                self.robot = None
                self.gripper = None

        return connected

    def reconnect(self):
        self.logger.info("Trying to reconnect")
        self.connect()
        while not self.is_connected():
            time.sleep(1)
            self.connect()

    def close(self):
        if self.is_running_policy():
            try:
                self.logger.debug("Terminating the current policy")
                state_log = self.robot.terminate_current_policy()
            except:
                self.logger.warn("Terminating the current policy failed")
            self.robot = None
            self.gripper = None
        return True

    def __del__(self):
        self.close()
