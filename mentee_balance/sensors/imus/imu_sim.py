import time

from rclpy.node import Node as RosNode
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu

from ..utils.math_utils import *
from .base import BaseIMU

GRAVITY = np.array([0, 0, -9.81])


class IMU_sim(BaseIMU):
    def __init__(self, name, cfg, ros_node: RosNode):
        super(IMU_sim, self).__init__(name, cfg, ros_node)
        self.seq = 0

    def imu_callback(self, msg: Imu):
        # Quaternion
        self.data[self.name].msg_frame_id = self.seq
        self.data[self.name].msg_stamp = time.time()
        self.data[self.name].quaternion = [0, 0, 0, 1]
        self.data[self.name].angular_velocity.lab = [0, 0, 0]
        self.data[self.name].angular_velocity.local = [0, 0, 0]
        self.data[self.name].acceleration.lab = [0, 0, 0]
        self.data[self.name].acceleration.local = [0, 0, 0]
        self.data[self.name].linear_velocity.lab = [0, 0, 0]
        self.data[self.name].linear_velocity.local = [0, 0, 0]
        self.seq += 1
        return self.name, self.data[self.name]
