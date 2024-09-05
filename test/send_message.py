# import zmq
# import json
# import time
# from simpub.xr_device.meta_quest3 import MetaQuest3InputData, MetaQuest3Hand

# context = zmq.Context()
# socket = context.socket(zmq.REQ)
# socket.connect("tcp://192.168.0.134:7721")

# register_info = {
#     "name": "ALRMetaQuest3",
#     "ip": "192.168.0.134",
#     "topics": [],
#     "services": [],
# }

# socket.send_string(f"Register:{json.dumps(register_info)}")
# message = socket.recv_string()
# print(message)

# pub_socket = context.socket(zmq.PUB)
# pub_socket.bind("tcp://192.168.0.134:7724")

# for _ in range(100):
#     new_data = MetaQuest3InputData()
#     right = MetaQuest3Hand()
#     right["pos"] = [0, 0, 0]
#     right["rot"] = [1, 0, 0, 0]
#     right["hand_trigger"] = True
#     right["index_trigger"] = True
#     new_data["A"] = False
#     new_data["X"] = False
#     new_data["Y"] = False
#     new_data["B"] = False
#     new_data["right"] = right
#     pub_socket.send_string(f"ALRMetaQuest3/InputData:{json.dumps(new_data)}")
#     time.sleep(0.01)

# new_data = MetaQuest3InputData()
# right = MetaQuest3Hand()
# right["pos"] = [0, 0, 0]
# right["rot"] = [1, 0, 0, 0]
# right["hand_trigger"] = False
# right["index_trigger"] = True
# new_data["right"] = right
# new_data["A"] = False
# new_data["X"] = False
# new_data["Y"] = False
# new_data["B"] = False
# pub_socket.send_string(f"ALRMetaQuest3/InputData:{json.dumps(new_data)}")
# time.sleep(0.01)

# new_data = MetaQuest3InputData()
# right = MetaQuest3Hand()
# right["pos"] = [0, 0, 0]
# right["rot"] = [1, 0, 0, 0]
# right["hand_trigger"] = False
# right["index_trigger"] = True
# new_data["right"] = right
# new_data["A"] = False
# new_data["X"] = True
# new_data["Y"] = False
# new_data["B"] = False
# pub_socket.send_string(f"ALRMetaQuest3/InputData:{json.dumps(new_data)}")
# time.sleep(0.01)
