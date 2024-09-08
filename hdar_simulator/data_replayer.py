import abc

from .recorder import RecordData


class DataReplayer(abc.ABC):

    def __init__(self, record_data_path: str):
        self.index = 0
        self.sequence_length = 0
        self.load_recod_data(record_data_path)

    def load_recod_data(self, record_data_path: str):
        self.record_data = RecordData()
        self.record_data.load_from_file(record_data_path)

    @abc.abstractmethod
    def create_simulator(self):
        pass

    @abc.abstractmethod
    def replay(self):
        while self.isSequenceFinished():
            self.replay_step()

    @abc.abstractmethod
    def replay_step(self):
        self.index += 1

    def isSequenceFinished(self):
        return self.index >= self.sequence_length
