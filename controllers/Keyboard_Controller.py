import numpy as np
import mujoco
import time
from .tcp_controller import InteractiveTCPControllerBase
from pynput import keyboard
from threading import Thread
from alr_sim.core import Scene
from tasks.task_manager import TaskManager


class KeyboardTCPController(InteractiveTCPControllerBase):
    def __init__(self, scene, robot, robot_config ):
        super().__init__(scene, robot, robot_config)
        self.desired_pos = np.array(robot_config["init_end_eff_pos"])
        self.desired_quat = np.array(robot_config["init_end_eff_quat"])
        self.step_size = 0.002  # 每次按键移动的距离
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
      
       

    def on_press(self, key):
        try:
            if key.char == '1':
                self.desired_pos[2] += self.step_size
            elif key.char == '2':
                self.desired_pos[2] -= self.step_size
            elif key.char == '3':
                self.desired_pos[0] += self.step_size
            elif key.char == '4':
                self.desired_pos[0] -= self.step_size
            elif key.char == '5':
                self.desired_pos[1] += self.step_size
            elif key.char == '6':
                self.desired_pos[1] -= self.step_size
            elif key.char == 'n':
                self.robot.open_fingers()
            elif key.char == 'm':
                self.robot.close_fingers(duration=0.0)
        except AttributeError:
            pass
   
    
            
       

    def read_ctrl_pos(self):
        return self.desired_pos

    def read_ctrl_quat(self):
        return self.desired_quat

    def start(self):
        try:
            while True:
                # 更新控制
                control_signal = self.getControl(self.robot)
                # 将控制信号应用于机器人
                self.robot.apply_control(control_signal)
                
                # 模拟场景步进
                self.scene.step()
        except KeyboardInterrupt:
            self.listener.stop()


# #tcp力
# # Load the model

# model=mujoco.MjModel.from_xml_path("./model/robot/gripper_panda.xml")
# data = mujoco.MjData(model)
# mujoco.mj_rnePostConstraint(model, data)

# # # Sampling interval (1 seconds)
# # sampling_interval = 1
# # last_sample_time = time.time()
# # # Run the simulation for a certain number of steps
# # for _ in range(10000):  # Adjust the number of steps as needed
# #     data = mujoco.MjData(model)
# #     # Check if it's time to sample
# #     current_time = time.time()
# #     if current_time - last_sample_time >= sampling_interval:
# #         # Get the force sensor data
# #         tcp_force_sensor_id = model.sensor_name2id('tcp_sensor')
# #         tcp_force_data = data.sensordata[tcp_force_sensor_id * 6 : (tcp_force_sensor_id + 1) * 6]
# #         # Print the force values
# #         print("Force: ", tcp_force_data)
# #         # Update the last sample time
# #         last_sample_time = current_time

# # Sampling interval (1 second)
# sampling_interval = 1
# last_sample_time = time.time()

# # Lists to store force data and time stamps
# force_data = []
# time_stamps = []

# # Run the simulation for a certain number of steps
# for step in range(50000):  # Adjust the number of steps as needed
#     mujoco.mj_step(model, data)

#     # Check if it's time to sample
#     current_time = time.time()
#     if current_time - last_sample_time >= sampling_interval:
#         # Get the force sensor data
#         tcp_body_id = model.body('tcp_sensor').id
#         tcp_force_torque_data = data.cfrc_ext[tcp_body_id] 
#         force_data = tcp_force_torque_data[:3]
#         # Append data to lists
#         force_data.append(force_data)
#         time_stamps.append(current_time)
        
#         # Update the last sample time
#         print(f"Time Step {time_stamps} - Force: {force_data}")
#         last_sample_time = current_time

# # Convert lists to numpy arrays
# force_data = np.array(force_data)
# time_stamps = np.array(time_stamps)
# if force_data.ndim == 1:
#     force_data = force_data.reshape(-1, 3)

# # # Plotting the force data over time
# # print("TCP Force Data: ", force_data)
# # plt.figure()
# # plt.plot(time_stamps, force_data[:, 0], label='Fx')
# # plt.plot(time_stamps, force_data[:, 1], label='Fy')
# # plt.plot(time_stamps, force_data[:, 2], label='Fz')
# # plt.xlabel('Time (s)')
# # plt.ylabel('Force (N)')
# # plt.title('TCP Force Sensor Data')
# # plt.legend()
# # plt.grid(True)
# # plt.show()