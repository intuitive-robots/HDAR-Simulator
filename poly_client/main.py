import poly_client, utils
import websockets
import asyncio
import json
import numpy as np
import torch
import threading
import time


robot_config = utils.get_real_robot_config()
robots = {}

for robot_name, config in robot_config.items():
    real_robot = poly_client.Panda(
        name=robot_name,
        ip=config["ip"],
        robot_port=config["robot_port"],
        gripper_port=config["gripper_port"],
    )
    real_controller = poly_client.RealRobotController(real_robot)
    robots[robot_name] = real_robot

def get_joint_msg():
    msg = {'Type': 'update_joints', 'data': {}}
    data = msg['data']
    for robot_name, robot in robots.items():
        data[robot_name] = {
            'joint_pos': list(map(float, robot.robot.get_joint_positions().numpy())),
            'joint_vel': list(map(float, robot.robot.get_joint_velocities().numpy())),
            'gripper_width': robot.gripper.get_state().width,
        }
    return str.encode(json.dumps(msg))

async def send_loop(websocket):
    while True:
        try:
            await websocket.send(get_joint_msg())
        except websockets.exceptions.ConnectionClosedOK:
            break
        except websockets.exceptions.ConnectionClosedError:
            break
        await asyncio.sleep(0.02)

async def recv_loop(websocket):
    async for msg in websocket:
        if json.loads(msg)['Type'] == 'shutdown':
            break
        await update_joints(msg)

async def update_joints(msg):
    reset_tasks = []
    data = json.loads(msg)['Data']
    for robot_name, robot_data in data.items():
        forces = robot_data['forces']
        forces = list(map(float, forces))
        forces = torch.tensor(np.array(forces))
        try:
            robots[robot_name].update_constraint_forces(forces)
        except:
            pass
    
        if robot_data['reset']:
            reset_tasks.append(reset_robot(robots[robot_name]))
    
    if len(reset_tasks) > 0:
        await asyncio.wait(reset_tasks, return_when=asyncio.ALL_COMPLETED)
    
async def reset_robot(robot):
    robot.robot.go_home(blocking=False)
    await asyncio.sleep(0.4)
    i = 0
    while not robot.is_running_policy() and i < 20:
        robot.robot.go_home(blocking=False)
        await asyncio.sleep(0.4)
        i += 1

    while robot.is_running_policy():
        await asyncio.sleep(0.1)

    robot.load_policy(blocking=False)
    i = 0
    while not robot.is_running_policy() and i < 20:
        robot.load_policy(blocking=False)
        await asyncio.sleep(0.4)
        i += 1

async def main():
    uri = "ws://localhost:8053"

    async with websockets.connect(uri) as websocket:
        await websocket.send(str.encode(json.dumps({'Type': 'start_stream'})))
        _, pending = await asyncio.wait(
            [
                asyncio.create_task(send_loop(websocket)),
                asyncio.create_task(recv_loop(websocket)),
            ],
            return_when=asyncio.FIRST_COMPLETED,
        )
    for robot in robots.values():
        if robot.is_running_policy():
            try:
                robot.robot.terminate_current_policy(return_log=False, timeout=1)
            except TimeoutError:
                pass


if __name__ == "__main__":
    asyncio.run(main())
