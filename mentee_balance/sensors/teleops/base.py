from abc import abstractmethod
from typing import Tuple

from .datatypes import CommandTwistData


class BaseCommand:

    def __init__(self, name, cfg, ros_node):
        self.name = name
        self.cfg = cfg

        # Data
        self.data = CommandTwistData()

    @abstractmethod
    def command_callback(self, msg) -> Tuple[str, CommandTwistData]:
        """IMU ros subscriber function"""
