import csv
import platform
import threading
import time
from contextlib import nullcontext
from pathlib import Path
from queue import Queue
from typing import Any, Dict, List

from rclpy.node import Node as RosNode


class BaseRecorder:
    def log_worker(self, event=None):
        """Flush all worker data to csv"""

    def record_row(self, row: Dict[str, str]):
        """Put new data into queue"""

    def flush(self):
        """Flush row buffer to queue"""

    def add_dict_to_buffer(self, data_dict: dict, append: bool = False):
        """Add dict to buffer"""

    def add_to_buffer(self, key: str, value: Any, append: bool = False):
        """Add new data to row_buffer"""

    def timeit(self, key: str, append: bool = False):
        """Creates context manager and time the action"""
        return nullcontext()


class Recorder(BaseRecorder):
    def __init__(self, file_name: str, ros_node, fields_names: List[str]):
        if platform.machine() == "x86_64":
            log_dir = Path.home().joinpath("log/latest/")
        else:
            log_dir = Path.home().joinpath("/data/log/latest/")
            #log_dir = Path("/data/log/latest/")
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        log_policy_path = Path.joinpath(log_dir, file_name)
        self.log_file_obj = open(log_policy_path, "w+")
        self.csv_writer = csv.DictWriter(self.log_file_obj, fieldnames=fields_names)
        self.csv_writer.writeheader()
        self.log_file_obj.flush()
        self.log_queue = Queue()
        if ros_node is not None:
            ros_node.create_timer(1 / 10, self.log_worker)
        else:
            RepeatedTimer(1 / 10, self.log_worker)

        # Row boffer
        self.row_buffer: Dict[str, str] = dict()

    def log_worker(self, event=None):
        queue_size = self.log_queue.qsize()
        if queue_size > 0:
            for _ in range(queue_size):
                log_dict = self.log_queue.get()
                self.csv_writer.writerow(log_dict)
                self.log_file_obj.flush()

    def record_row(self, row: Dict[str, str]):
        self.log_queue.put(row)

    def flush(self):
        if len(self.row_buffer) > 0:
            buffer = {key: str(value) for key, value in self.row_buffer.items()}
            self.log_queue.put(buffer)
        self.row_buffer = dict()

    def add_dict_to_buffer(self, data_dict: dict, append: bool = False):
        for key, value in data_dict.items():
            self.add_to_buffer(key, value, append)

    def add_to_buffer(self, key: str, value: Any, append: bool = False):
        if append and key in self.row_buffer:
            self.row_buffer[key] += value
        else:
            self.row_buffer[key] = value

    def timeit(self, key: str, append: bool = False):
        return TimeIt(self, key, append)

    def __del__(self):

        self.log_file_obj.flush()


class TimeIt:
    def __init__(self, recorder: Recorder, key: str, append: bool = False) -> None:
        self.recorder = recorder
        self.key = key
        self.append = append
        self.start_time = time.time()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        t = time.time() - self.start_time
        t = [t] if self.append else t
        self.recorder.add_to_buffer(self.key, t, self.append)

        return True


class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.next_call = time.time()
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self.next_call += self.interval
            self._timer = threading.Timer(self.next_call - time.time(), self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False
