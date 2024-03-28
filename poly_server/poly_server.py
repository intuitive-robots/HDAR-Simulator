import asyncio
import json
import threading
from typing import List
from websockets import server

from alr_sim.core.Robots import RobotBase
from alr_sim.core.Scene import Scene


class PolyServer:
    def __init__(
        self,
        robots: List[RobotBase],
        vt_controllers,
        scene: Scene,
        host="127.0.0.1",
        port=8053,
    ) -> None:

        self.robots = robots
        self.scene = scene
        self.vt_controllers = vt_controllers

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
            update_joints=self.update_joints
        )
        self.first = True
    
    async def update_joints(self, msg):
        data = msg['data']
        for robot_name, joint_info in data.items():
            controller = self.vt_controllers[robot_name]
            controller.update_real_joints(joint_info)
    
    async def start_stream(self, *args, **kwargs):
        print("Start stream to client")
        self.on_stream = True

    async def close_stream(self, *args, **kwargs):
        print("Close stream to client")
        self.on_stream = False

    def register_callback(self, **kwargs):
        for k, cb in kwargs.items():
            self.register_callback_dict[k] = cb

    async def execute_callback(self, request: dict):
        assert type(request) is dict
        request_type = request["Type"]
        if request_type in self.register_callback_dict.keys():
            callback = self.register_callback_dict[request_type]
        else:
            print(f"Wrong Request Type for {request_type}!!!")
            return
        await callback(request)

    def shutdown(self):
        self.send_dict({'Type': 'shutdown'})
        self.connected = False
        if self.wsserver is not None:
            self.wsserver.close()
        if self.server_future is not None:
            self.server_future.set_result(True)
    
    def get_msg(self):
        msg = {
            "Type": "constraint_forces",
            "Data": {},
        }
        data: dict = msg["Data"]
        for robot in self.robots:
            forces = [
                -self.scene.data.joint(name).qfrc_smooth[0]
                for name in robot.joint_names
            ]
            controller = self.vt_controllers[robot.name] 
            reset_flag = controller.reset_flag
            if reset_flag:
                controller.reset_flag = False
            data[robot.name] = {'forces': forces, 'reset': reset_flag}
        return msg

    async def stream_handler(self, ws: server.WebSocketServerProtocol):
        while self.connected:
            if not self.on_stream:
                await asyncio.sleep(0.02)
                continue
            msg = self.get_msg()
            try:
                await self._send_dict_msg(msg, ws, 0.02)
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
