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
        record_mode: bool,
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
        # recording flag
        self.on_recording = False
        # counter for the demonstration and frames
        self.demo_counter = 0
        self.skip_counter = 0

    def start_record(self):
        if not self.record_mode or self.on_recording:
            return
        logger.info("Start recording")
        self.skip_counter = 0
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
        mp.Process(
            target=self._save_record, kwargs={"file_name": file_name}
        ).start()
        logger.info(f"Saving record to {file_name}")
        self.demo_counter += 1

    @abc.abstractmethod
    def _start_record(self):
        raise NotImplementedError

    @abc.abstractmethod
    def _save_record(self, file_name: str):
        logger.info(f"Finishing saving record to {file_name}")

    @abc.abstractmethod
    def _record(self):
        raise NotImplementedError

    def record(self):
        if not self.record_mode or not self.on_recording:
            return
        if self.skip_counter % self.record_steps == 0:
            self._record()
            self.skip_counter = 0
            return
        self.skip_counter += 1
