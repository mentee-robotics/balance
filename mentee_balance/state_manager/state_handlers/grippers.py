import logging
import time
from abc import abstractmethod
from collections import defaultdict
from functools import partial
from typing import Callable, Dict, List, Optional, Tuple, Type

from candle_ros2.msg import MotionCommand

from mentee_balance.sensors.gripper.datatypes import GripperData

from ...sensors import gripper
from ...sensors.gripper import BaseGripper, GripperData
from ...utils.recorder import Recorder
from .base import BaseManager
# from .datatypes import GripperData

logger = logging.getLogger("menteebot")


class BaseGripperStateManager(BaseManager):
    def __int__(self, cfg):
        super(BaseGripperStateManager, self).__init__(cfg)
        self.grippers: Dict[str, BaseGripper] = dict()

        for name, gripper_cfg in self.cfg.grippers.items():
            print(name , gripper_cfg)

