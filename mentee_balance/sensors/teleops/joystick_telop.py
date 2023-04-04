import logging

from common.msg import MenteeCmd
from rclpy.node import Node as RosNode
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rich.console import Console
from std_msgs.msg import String

from .base import BaseCommand
from .datatypes import CommandData

console = Console()
logger = logging.getLogger("menteebot")


class JoystickTeleop(BaseCommand):
    def __init__(self, name, cfg, ros_node: RosNode):
        super(JoystickTeleop, self).__init__(name, cfg, ros_node)
        self.data = CommandData()
        subs_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        )
        self.joystick_commander = ros_node.create_subscription(
            String, "/joystick/command_on", self.joystick_on_callback, subs_qos
        )
        self.command_on = True
        self.subscribers = ros_node.create_subscription(
            MenteeCmd, self.cfg.topic_name, self.command_callback, subs_qos
        )

    def command_callback(self, msg: MenteeCmd):
        if self.command_on:
            msg_stamp = msg.header.stamp.sec + (1e-9 * msg.header.stamp.nanosec)
            self.data.msg_stamp = msg_stamp
            self.data.msg_frame_id = msg.header.frame_id
            self.data.lin_vel_x = msg.lin_vel_x
            self.data.lin_vel_y = msg.lin_vel_y
            self.data.heading = msg.heading
            self.data.foot_height = msg.foot_height

        return self.data

    def joystick_on_callback(self, msg):
        case = msg.data
        if case == "command_off":
            self.command_on = False
            logger.info("Joystick OFF")

        elif case == "command_on":
            self.command_on = True
            logger.info("Joystick ON")
