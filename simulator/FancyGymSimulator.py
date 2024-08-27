import time
import fancy_gym
from simpub.sim.fancy_gym import FancyGymPublisher

from ..recoder import FancyGymRecorder


class FancyGymSimulator:

    def __init__(self, env_name: str):
        self.env_name = env_name
        self.env = fancy_gym.make(env_name, seed=1)
        self.publisher = FancyGymPublisher(env_name, self.env, "127.0.0.1")
        self.recorder = FancyGymRecorder(env_name)

    def run(self):
        self.env.reset()
        while True:
            action = self.env.action_space.sample()
            _, _, done, _ = self.env.step(action)
            self.recorder.record()
            self.env.render()
            time.sleep(0.01)
