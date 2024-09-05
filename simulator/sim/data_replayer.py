import abc
import pickle


class DataReplayer(abc.ABC):

    def __init__(self):
        self.index = 0
        self.sequence_length = 0

    def load_data(self, data_path: str):
        data = pickle.load(open(data_path, "rb"))
        self.create_simulator(data)

    @abc.abstractmethod
    def create_simulator(self):
        pass

    def replay_data(self):
        while True:
            self.replay_step()

    @abc.abstractmethod
    def start_replay(self):
        pass

    @abc.abstractmethod
    def replay_step(self):
        self.index += 1

    def isSequenceFinished(self):
        return self.index >= self.sequence_length