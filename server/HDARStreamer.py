from ast import Dict
import asyncio
import json
import threading
from websockets import server
import mujoco

from alr_sim.sims.mj_beta import MjScene
from alr_sim.core.Robots import RobotBase
from alr_sim.core.sim_object import SimObject
from alr_sim.sims.universal_sim.PrimitiveObjects import Box, Sphere

from sim_pub.sf import SFSimStreamer
from sim_pub.sf.sf_simobj_publisher import *

class HDARStreamer(SFSimStreamer):
    def __init__(
        self,
        server_config: Dict
    ) -> None:

        self.ws: server.WebSocketServerProtocol
        self.wsserver: server.WebSocketServer 
        self.server_future: asyncio.Future
        self.host = server_config["server_addr"]
        self.port = server_config["server_port"]

        # flags
        self.connected = False
        self.on_stream = False

        # defaule register function
        self.register_callback_dict = dict()
        self.register_callback(
            start_stream=self.start_stream,
            close_stream=self.close_stream,
        )

        self.interaction_object_list = list()

        hand_list = [
            "LeftHand",
            "RightHand",
        ]

        hand_joint_list = [
            "Wrist",
            "Palm",
            "ThumbMetacarpalJoint",
            "ThumbProximalJoint",
            "ThumbDistalJoint",
            "ThumbTip",
            "IndexMetacarpalJoint",
            "IndexKnuckle",
            "IndexMiddleJoint",
            "IndexDistalJoint",
            "IndexTip",
            "MiddleMetacarpalJoint",
            "MiddleKnuckle",
            "MiddleMiddleJoint",
            "MiddleDistalJoint",
            "MiddleTip",
            "RingMetacarpalJoint",
            "RingKnuckle",
            "RingMiddleJoint",
            "RingDistalJoint",
            "RingTip",
            "PinkyMetacarpalJoint",
            "PinkyKnuckle",
            "PinkyMiddleJoint",
            "PinkyDistalJoint",
            "PinkyTip",
        ]

        for hand_name in hand_list:
            for joint_name in hand_joint_list:
                new_interaction_obj = Sphere(
                    hand_name + joint_name,
                    [0, 0, 0],
                    [1, 0, 0, 0],
                    size=[0.01],
                    rgba=[0, 0, 1, 0.0],
                    static=True,
                    visual_only=True,
                )
                self.interaction_object_list.append(new_interaction_obj)
                self.scene.add_object(new_interaction_obj)

        new_interaction_obj = Sphere(
            "gaze_hit_point",
            [0, 0, 0],
            [1, 0, 0, 0],
            size=[0.05],
            rgba=[1, 0, 0, 0.0],
            static=True,
            visual_only=True,
        )
        self.interaction_object_list.append(new_interaction_obj)
        self.scene.add_object(new_interaction_obj)

        new_interaction_obj = Box(
            "user_head_pose",
            [0, 0, 0],
            [1, 0, 0, 0],
            size=[0.1, 0.1, 0.1],
            rgba=[0, 1, 0, 0.0],
            static=True,
            visual_only=True,
        )
        self.interaction_object_list.append(new_interaction_obj)
        self.scene.add_object(new_interaction_obj)

        # HoloLens Control Data
        self.tcp_pos = None
        self.tcp_quat = None
        self.register_callback(manipulate_objects=self.update_manipulated_object)
        # self.manipulated_message = None
