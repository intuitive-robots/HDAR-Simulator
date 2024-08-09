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

