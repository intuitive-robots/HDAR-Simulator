import abc
import pickle


class DataReplayer(abc.ABC):

    def load_data(self, data_path: str):
        data = pickle.load(open(data_path, "rb"))
        self.create_simulation(data)

    @abc.abstractmethod
    def create_simulation(self):
        pass

    def replay_data(self):
        while True:
            self.replay_step()

    @abc.abstractmethod
    def start_replay(self):
        pass

    @abc.abstractmethod
    def replay_step(self):
        pass
