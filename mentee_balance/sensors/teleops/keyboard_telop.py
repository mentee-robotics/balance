import time
import logging
from rich.console import Console
from rclpy.node import Node as RosNode
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

from .base import BaseCommand

console = Console()
logger = logging.getLogger("menteebot")


class KeyboardTeleop(BaseCommand):
    def __init__(self, name, cfg, ros_node: RosNode):
        super(KeyboardTeleop, self).__init__(name, cfg, ros_node)
        subs_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        )
        self.keyboard_commander = ros_node.create_subscription(
            String, "/keyboard/command_on", self.keyboard_on_callback, subs_qos
        )
        self.command_on = True
        self.subscribers = ros_node.create_subscription(TwistStamped, self.cfg.topic_name, self.command_callback, subs_qos)

    def command_callback(self, msg: TwistStamped):
        curr_time = time.time()
        if self.command_on:
            msg_stamp = msg.header.stamp.sec + (1e-9 * msg.header.stamp.nanosec)
            self.data.msg_stamp = msg_stamp
            self.data.msg_frame_id = msg.header.frame_id
            self.data.linear_vel_x = msg.twist.linear.x
            self.data.linear_vel_y = msg.twist.linear.y
            self.data.linear_vel_z = msg.twist.linear.z
            self.data.angular_vel_x = msg.twist.angular.x
            self.data.angular_vel_y = msg.twist.angular.y
            self.data.angular_vel_z = msg.twist.angular.z

        return self.data

    def keyboard_on_callback(self, msg):
        case = msg.data
        if case == "command_off":
            self.command_on = False
            logger.info("Keyboard OFF")

        elif case == "command_on":
            self.command_on = True
            logger.info("keyboard ON")
