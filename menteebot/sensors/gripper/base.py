import logging
import math
from abc import abstractmethod
from collections import defaultdict
from typing import Dict, Optional

import numpy as np

logger = logging.getLogger("menteebot")


class BaseGripper:
    def __init__(self, name, cfg):
        self.name = name
        self.cfg = cfg


    def get_data(self):  # -> Tuple[str, dict[str, float]]
        """Generate current data for each motor"""


    def set_data(self, req_id: int, grippers_command):  # motors_command: dict[str, dict[str, float]])
        """Apply action to motor"""
