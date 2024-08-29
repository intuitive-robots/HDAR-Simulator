import abc
import os
from datetime import datetime

from simpub.core.log import logger
import multiprocessing as mp

class Recorder(abc.ABC):

    def __init__(
        self,
        task_name: str,
        save_root_path: str,
        record_mode=False,
        record_steps=1,
    ) -> None:
        super().__init__()
        # name the demonstration by date and time
        record_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.task_name = task_name
        self.save_path = os.path.join(
            save_root_path,
            f"{task_name}_{record_time}")
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        self.record_mode = record_mode
        self.record_steps = record_steps
        self.on_recording = False
        self.demo_counter = 0

    def start_record(self):
        if not self.record_mode or self.on_recording:
            return
        logger.info("Start recording")
        self.on_recording = True

    def stop_record(self):
        if not self.record_mode or not self.on_recording:
            return
        logger.info("Stop recording")
        self.on_recording = False

    def save_record(self):
        if not self.record_mode:
            return
        self.stop_record()
        file_name = "{}_{:03d}.pkl".format(self.task_name, self.demo_counter)
        # process = mp.Process(
        #     target=self._save_record, kwargs={"file_name": file_name}
        # ).start()
        # process.join()
        self._save_record(file_name)
        self.demo_counter += 1

    @abc.abstractmethod
    def _save_record(self, file_name: str):
        pass

    @abc.abstractmethod
    def record(self, data):
        pass
