import logging
import math
from abc import abstractmethod
from collections import defaultdict
from typing import Dict, Optional

import numpy as np
from rclpy.node import Node as RosNode

logger = logging.getLogger("menteebot")


class BaseMotor:
    def __init__(self, name, cfg, ros_node: RosNode = None):
        self.name = name
        self.cfg = cfg
        self.ros_node = ros_node
        self.enable = False

        # Joint params
        self.joints_params = dict(self.cfg.joints_params)
        # TODO: need to find the right place to put the coeffs
        self.joints_params.pop("savgol_coeffs", None)
        self.joints_params.pop("candle_params", None)

        # Get mapping
        self.motor_id2rigid_body = dict(self.cfg.mapping.items())
        self.rigid_body2motor_id = dict(map(reversed, self.motor_id2rigid_body.items()))

        # Get init position
        self.init_positions = dict(cfg.init_position)
        self.candle_params = cfg.joints_params.get("candle_params", [])
        self.savgol_coeffs = cfg.joints_params.get("savgol_coeffs", [])
        self.candle_params = cfg.joints_params.get("candle_params", [])
        self.pid_params = cfg.get("pid_params", None)
        # Init defaults
        self.default_kp = defaultdict(float)
        self.default_kd = defaultdict(float)
        self.init_kp = defaultdict(float)
        self.init_kd = defaultdict(float)
        self.max_torque = defaultdict(float)
        self.watchdog_params = defaultdict(dict)
        self.kalman_filter_params = defaultdict(dict)
        for rigid_body_name, params in self.joints_params.items():
            motor_id = self.rigid_body2motor_id[rigid_body_name]
            init_params = params.init_params
            self.init_kp[motor_id] = init_params.init_kp
            self.init_kd[motor_id] = init_params.init_kd
            self.default_kp[motor_id] = init_params.default_kp
            self.default_kd[motor_id] = init_params.default_kd
            self.max_torque[motor_id] = init_params.default_max_torque
            self.watchdog_params[motor_id] = dict(params.watchdog)
            kalman_filter_params = dict(params.get("kalman_filter", {}))
            if kalman_filter_params:
                self.kalman_filter_params[motor_id] = kalman_filter_params

        self.boot()
        self.go_to_init()

    @abstractmethod
    def boot(self):
        """Initialise motors and specific class variables"""

    @abstractmethod
    def go_to_init(self):
        """Move motors to init position"""

    @abstractmethod
    def get_data(self, **kwargs):  # -> Tuple[str, dict[str, float]]
        """Generate current data for each motor"""

    @abstractmethod
    def apply(self, req_id: int, motors_command):  # motors_command: dict[str, dict[str, float]])
        """Apply action to motor"""

    @abstractmethod
    def _get_single_dof_data(self, motor_id, state):
        """This is a helper function to watchdog to get current data of motor"""
