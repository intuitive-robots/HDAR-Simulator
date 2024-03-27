import asyncio
import json
import threading
from typing import List
from websockets import server

from alr_sim.core.Scene import Scene
from alr_sim.core.Robots import RobotBase
from alr_sim.core.sim_object import SimObject

from utils import mj2unity_size, mj2unity_pos, mj2unity_quat


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
                # "static": 0 if not hasattr(obj, "rgba") else 1,
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


class RobotHandler:
    count = 0

    def __init__(self, robot: RobotBase, scene: Scene) -> None:
        self.robot = robot
        self.scene = scene

        if hasattr(robot, "name"):
            self.name = getattr(robot, "name")
        else:
            self.name = f"panda{RobotHandler.count}"
            RobotHandler.count += 1
        self.type = getattr(robot, "type", "gripper_panda")

    def get_robot_param_dict(self) -> dict:
        param_date = {}
        param_date["pos"] = list(mj2unity_pos(self.robot.base_position))
        param_date["rot"] = list(mj2unity_quat(self.robot.base_orientation))
        joints = list(self.robot.current_j_pos)
        joints.extend([self.robot.gripper_width / 2, self.robot.gripper_width / 2])
        param_date["joints"] = joints
        return {
            "attr": {
                "type": self.type,
                "interaction_method": getattr(self.robot, "interaction_method"),
            },
            "data": param_date,
        }

    def get_robot_state_dict(self) -> dict:
        joints = list(self.robot.current_j_pos)
        joints.extend([self.robot.gripper_width / 2, self.robot.gripper_width / 2])
        return {
            "data": {
                "joints": joints,
            }
        }


class UnityStreamer:
    def __init__(
        self,
        scene: Scene,
        robots: List[RobotBase],
        objects: List[SimObject],
        host="127.0.0.1",
        port=8052,
    ) -> None:

        self.scene = scene
        if robots is not None:
            self.robot_handlers = [RobotHandler(robot, scene) for robot in robots]
        if objects is not None:
            self.object_handlers = [ObjectHandler(obj, scene) for obj in objects]

        self.ws: server.WebSocketServerProtocol = None
        self.wsserver: server.WebSocketServer = None
        self.server_future: asyncio.Future = None
        self.host = host
        self.port = port

        # flags
        self.connected = False
        self.on_stream = False

        # defaule register function
        self.register_callback_dict = dict()
        self.register_callback(
            start_stream=self.start_stream,
            close_stream=self.close_stream,
        )

        # self.interaction_object_list = list()

        # hand_list = [
        #     "LeftHand",
        #     "RightHand",
        # ]

        # hand_joint_list = [
        #     "Wrist",
        #     "Palm",
        #     "ThumbMetacarpalJoint",
        #     "ThumbProximalJoint",
        #     "ThumbDistalJoint",
        #     "ThumbTip",
        #     "IndexMetacarpalJoint",
        #     "IndexKnuckle",
        #     "IndexMiddleJoint",
        #     "IndexDistalJoint",
        #     "IndexTip",
        #     "MiddleMetacarpalJoint",
        #     "MiddleKnuckle",
        #     "MiddleMiddleJoint",
        #     "MiddleDistalJoint",
        #     "MiddleTip",
        #     "RingMetacarpalJoint",
        #     "RingKnuckle",
        #     "RingMiddleJoint",
        #     "RingDistalJoint",
        #     "RingTip",
        #     "PinkyMetacarpalJoint",
        #     "PinkyKnuckle",
        #     "PinkyMiddleJoint",
        #     "PinkyDistalJoint",
        #     "PinkyTip",
        # ]

        # for hand_name in hand_list:
        #     for joint_name in hand_joint_list:
        #         new_interaction_obj = Sphere(
        #             hand_name + joint_name,
        #             [0, 0, 0],
        #             [1, 0, 0, 0],
        #             size=[0.01],
        #             rgba=[0, 0, 1, 0.4],
        #             static=True,
        #             visual_only=True,
        #         )
        #         self.interaction_object_list.append(new_interaction_obj)
        #         self.scene.add_object(new_interaction_obj)

        # new_interaction_obj = Sphere(
        #     "gaze_hit_point",
        #     [0, 0, 0],
        #     [1, 0, 0, 0],
        #     size=[0.05],
        #     rgba=[1, 0, 0, 0.4],
        #     static=True,
        #     visual_only=True,
        # )
        # self.interaction_object_list.append(new_interaction_obj)
        # self.scene.add_object(new_interaction_obj)

        # new_interaction_obj = Box(
        #     "user_head_pose",
        #     [0, 0, 0],
        #     [1, 0, 0, 0],
        #     size=[0.1, 0.1, 0.1],
        #     rgba=[0, 1, 0, 0.4],
        #     static=True,
        #     visual_only=True,
        # )
        # self.interaction_object_list.append(new_interaction_obj)
        # self.scene.add_object(new_interaction_obj)

        # HoloLens Control Data
        self.tcp_pos = None
        self.tcp_quat = None
        self.register_callback(manipulate_objects=self.update_manipulated_object)
        # self.manipulated_message = None

    def update_manipulated_object(self, msg):
        pass
        # for id, obj_data in msg["manipulationData"].items():
        #     body_id = mj_name2id(self.scene.model, mjtObj.mjOBJ_BODY, id)
        #     if obj_data is None:
        #         self.scene.model.body_gravcomp[body_id] = 0
        #         continue
        #     data = obj_data["data"]
        #     self.scene.set_obj_pos_and_quat(data["pos"], data["rot"], obj_name=id)
        #     self.scene.model.body_gravcomp[body_id] = 1

    # def load(self, robots: UnityRobot, scene: UnityScene):
    #     # collecte the data from robot and transform them to dict
    #     self.robots, self.scene = robots, scene

    def start_stream(self, *args, **kwargs):
        print("Start stream to client")
        self.send_dict(self.getInitParamDict())
        self.on_stream = True

    def close_stream(self, *args, **kwargs):
        print("Close stream to client")
        self.on_stream = False

    def register_callback(self, **kwargs):
        for k, cb in kwargs.items():
            self.register_callback_dict[k] = cb

    async def execute_callback(self, request: dict):
        # print("Received a {} request".format(request["Type"]))
        assert type(request) is dict
        # if type(request) is str:
        #     print(request)
        #     return
        request_type = request["Type"]
        if request_type in self.register_callback_dict.keys():
            callback = self.register_callback_dict[request_type]
        else:
            print(f"Wrong Request Type for {request_type}!!!")
            return
        callback(request)

    def shutdown(self):
        self.connected = False
        if self.wsserver is not None:
            self.wsserver.close()
        if self.server_future is not None:
            self.server_future.set_result(True)
        # self.wsserver.get_loop().stop()

    async def stream_handler(self, ws: server.WebSocketServerProtocol):
        # assert self.robots is not None and self.scene is not None
        while self.connected:
            # print(time.time() - self.last_time)
            # self.last_time = time.time()
            if not self.on_stream:
                await asyncio.sleep(0.1)
                continue
            new_msg = self.getStateMsg()
            try:
                await self._send_dict_msg(new_msg, ws, 0.02)
            except:
                print("error occured when sending messages!!!!!")
                self.connected = False
                await ws.close()
                break
            finally:
                pass
        print("finish the stream handler")

    async def request_handler(self, ws: server.WebSocketServerProtocol):
        async for request in ws:
            # print(request)
            await self.execute_callback(json.loads(request))
        print("finish the request handler")

    async def handler(self, ws: server.WebSocketServerProtocol):
        print(f"connected by: {ws.local_address}")
        self.ws = ws
        self.connected = True
        # self.last_time = time.time()
        assert self.robot_handlers is not None
        _, pending = await asyncio.wait(
            [
                asyncio.create_task(self.stream_handler(ws)),
                asyncio.create_task(self.request_handler(ws)),
            ],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()
        self.connected = False
        await self.ws.close()
        print(f"the connection to {ws.local_address} is closed")

    async def expect_client(self):
        self.server_future = asyncio.Future()
        async with server.serve(self.handler, self.host, self.port) as self.wsserver:
            # TODO: Is there another good way to do it?
            while not self.server_future.done():
                await asyncio.sleep(1)
            # self.wsserver.get_loop().stop()
            # await self.wsserver.serve_forever()

    def start_server_task(self):
        print(f"streamer has been started in {self.host}:{self.port}")
        asyncio.run(self.expect_client())
        print("server task finished")

    def close_server(self):
        self.connected = False

    def start(self, block=False):
        threading.Thread(target=self.start_server_task).start()
        while block and not self.connected:
            pass

    async def _send_dict_msg(self, msg: str, ws: server.WebSocketServerProtocol, t=0):
        await ws.send(str.encode(json.dumps(msg)))
        await asyncio.sleep(t)

    def send_dict(self, msg, t=0):
        # loop = asyncio.get_event_loop()
        # loop.run_until_complete(self._send_dict_msg(msg, self.ws, t))
        if not self.connected:
            return
        loop = self.wsserver.get_loop()
        loop.create_task(self._send_dict_msg(msg, self.ws, t))

    def send_message(self, msg: str):
        self.send_dict(
            {
                "Header": "text_message",
                "TextMessage": msg,
            }
        )

    def getInitParamDict(self):
        msg = {
            "Header": "initial_parameter",
            "Data": {},
        }
        data: dict = msg["Data"]
        for robot_handler in self.robot_handlers:
            data[robot_handler.name] = robot_handler.get_robot_param_dict()
        for obj_handler in self.object_handlers:
            data[obj_handler.name] = obj_handler.get_obj_param_dict()

        return msg

    def getStateMsg(self):
        msg = {
            "Header": "stream_data",
            "Data": {},
        }
        data: dict = msg["Data"]
        for robot_handler in self.robot_handlers:
            data[robot_handler.name] = robot_handler.get_robot_state_dict()
        for obj_handler in self.object_handlers:
            data[obj_handler.name] = obj_handler.get_obj_state_dict()
        return msg
