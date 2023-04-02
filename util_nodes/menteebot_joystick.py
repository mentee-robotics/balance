#!/usr/bin/env python
import argparse
import logging

import numpy as np
import rclpy
from common.msg import MenteeCmd
from common.srv import BrainStatus, TriggerListService
from hydra import compose, initialize
from omegaconf import OmegaConf
from rich.console import Console
from rich.live import Live
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

from .control_panel import ControlPanel

logger = logging.getLogger("menteebot")


class Joystick:
    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument(
            "--yaml", default="../configs/sim_network_no_ros.yaml", help="Configuration file"
        )

        args, unknown = self.parser.parse_known_args()

        # Configuring hydra
        yaml_path_split = args.yaml.split("/")
        config_path = "/".join(yaml_path_split[:-1])
        config_name = yaml_path_split[-1][:-5]
        with initialize(config_path=config_path, job_name="menteebot_app"):
            yaml_conf = compose(config_name=config_name, overrides=["hydra.run.dir=/tmp"])
            # Struct to normal :)
            yaml_conf = OmegaConf.to_container(yaml_conf)
            yaml_conf = OmegaConf.create(yaml_conf)

        cfg = yaml_conf

        # Publisher topic name
        topic_name = cfg.sensors.teleops.joystick_commands.topic_name
        self.sin_net = False
        # Commands
        self.network_cfg = cfg.networks
        self.commands_params_cfg = self.network_cfg.main.command_dict

        # Default values
        if not self.sin_net:
            self.lin_vel_x = self.commands_params_cfg.default_value.get("lin_vel_x", 0.0)
            self.lin_vel_y = self.commands_params_cfg.default_value.get("lin_vel_y", 0.0)
            self.heading = self.commands_params_cfg.default_value.get("heading", 0.0)
        else:
            self.lin_vel_x = self.commands_params_cfg.default_value.get("sin_amp", 0.0)
            self.lin_vel_y = self.commands_params_cfg.default_value.get("sin_omega", 0.0)
            self.heading = self.commands_params_cfg.default_value.get("dof_selector", 0.0)
            self.prev_heading = self.commands_params_cfg.default_value.get("dof_selector", 0.0)
        self.special = self.commands_params_cfg.default_value.get("foot_height", 0.0)
        self.brain_main_loop = False
        self.go_to_init = False
        self.network = "main"
        self.bml_res = False
        self.next_network = self.network
        self.network_list = list(cfg.networks.keys())
        self.next_network_index = self.network_list.index(self.network)
        self.network_index = self.network_list.index(self.network)

        # Ranges
        if not self.sin_net:
            self.range_lin_vel_x = self.commands_params_cfg.ranges.get(
                "lin_vel_x", [float("-inf"), float("inf")]
            )
            self.range_lin_vel_y = self.commands_params_cfg.ranges.get(
                "lin_vel_y", [float("-inf"), float("inf")]
            )
            self.range_heading = self.commands_params_cfg.ranges.get(
                "heading", [float("inf"), float("-inf")]
            )
        else:
            self.range_lin_vel_x = self.commands_params_cfg.ranges.get(
                "sin_amp", [float("-inf"), float("inf")]
            )
            self.range_lin_vel_y = self.commands_params_cfg.ranges.get(
                "sin_omega", [float("-inf"), float("inf")]
            )
            self.range_heading = self.commands_params_cfg.ranges.get(
                "dof_selector", [float("inf"), float("-inf")]
            )
        self.range_special = self.commands_params_cfg.ranges.get(
            "foot_height", [float("-inf"), float("inf")]
        )

        # Increment step
        if not self.sin_net:
            self.step_x = self.commands_params_cfg.increment.get("lin_vel_x", 0.0)
            self.step_y = self.commands_params_cfg.increment.get("lin_vel_y", 0.0)
            self.step_heading = self.commands_params_cfg.increment.get("heading", 0.0)
        else:
            self.step_x = self.commands_params_cfg.increment.get("sin_amp", 0.0)
            self.step_y = self.commands_params_cfg.increment.get("sin_omega", 0.0)
            self.step_heading = self.commands_params_cfg.increment.get("dof_selector", 0.0)
        self.step_special = self.commands_params_cfg.increment.get("foot_height", 0.0)

        # Control Panel
        self.cp = ControlPanel(
            active_policy=self.network,
            policies=self.network_list,
            main_loop=False,
            go_to_init=self.go_to_init,
            cmd_x=self.lin_vel_x,
            cmd_y=self.lin_vel_y,
            cmd_h=self.heading,
            cmd_fh=self.special,
            sin_mode=self.sin_net,
        )

        # Init frame ID
        self.frame_id = 0

        # Init ROS2
        rclpy.init()

        # Create ROS node
        self.node = rclpy.create_node("teleop_joystick")

        # Create publisher with topic-name
        self.pub = self.node.create_publisher(MenteeCmd, topic_name, 10)
        # Subscribe to joystick
        self.joy_sub = self.node.create_subscription(Joy, "/joy", self.listener_callback, 10)

        # Brain services
        self.brain_main_loop = self.node.create_client(SetBool, "net_loop")
        while not self.brain_main_loop.wait_for_service(timeout_sec=1.0):
            logger.info('Service "Brain - main_loop" not available, waiting again...')
        self.net_start = SetBool.Request()
        self.brain_change_net = self.node.create_client(TriggerListService, "change_net")
        while not self.brain_change_net.wait_for_service(timeout_sec=1.0):
            logger.info('Service "Brain - change_net" not available, waiting again...')
        self.net_req = TriggerListService.Request()
        self.brain_get_status = self.node.create_client(BrainStatus, "brain_status")
        while not self.brain_get_status.wait_for_service(timeout_sec=1.0):
            logger.info('Service "Brain - brain_status" not available, waiting again...')
        self.stat = BrainStatus.Request()

        # Dashboard
        self.console = Console()
        self.live = Live(self.cp.layout, console=self.console)
        self.live.start()

    def listener_callback(self, msg: Joy):
        # Update continuous parameters
        self.lin_vel_x += (1 - np.abs(self.cp.commands_panel.x_click)) * msg.axes[1] * self.step_x
        self.lin_vel_y += (1 - np.abs(self.cp.commands_panel.y_click)) * msg.axes[6] * self.step_y
        self.heading += (
            (1 - np.abs(self.cp.commands_panel.h_click)) * msg.axes[0] * self.step_heading
        )
        if not self.sin_net:
            self.heading = self.wrap_to_pi(self.heading)
        else:
            if self.heading != self.prev_heading:
                self.reset_dof_selector(self.network)
                self.prev_heading = self.heading

        self.special += (
            (1 - np.abs(self.cp.commands_panel.fh_click)) * msg.axes[7] * self.step_special
        )

        # Clip parameters
        self.lin_vel_x = np.clip(
            self.lin_vel_x, a_min=self.range_lin_vel_x[0], a_max=self.range_lin_vel_x[1]
        )
        self.lin_vel_y = np.clip(
            self.lin_vel_y, a_min=self.range_lin_vel_y[0], a_max=self.range_lin_vel_y[1]
        )
        self.heading = np.clip(
            self.heading, a_min=self.range_heading[0], a_max=self.range_heading[1]
        )
        self.special = np.clip(
            self.special, a_min=self.range_special[0], a_max=self.range_special[1]
        )

        # Update binary parameters
        # Start/Stop main brain loop
        if msg.buttons[0] == 1:  # (A)
            # self.brain_main_loop = True
            self.bml_res = True
            self.set_net_loop(self.bml_res)
        if msg.buttons[1] == 1:  # (B)
            # self.brain_main_loop = False
            self.bml_res = False
            self.set_net_loop(self.bml_res)

        # Select policy
        if msg.buttons[4] == 1 and not self.cp.policies_panel.sp_click:  # LB
            self.next_network_index -= 1
        if msg.buttons[5] == 1 and not self.cp.policies_panel.sp_click:  # RB
            self.next_network_index += 1
        self.next_network_index = self.next_network_index % len(self.network_list)

        # Apply policy
        if msg.buttons[2] == 1:  # (X)
            self.network = self.network_list[self.next_network_index]
            self.network_index = self.next_network_index
            self.change_network(self.network)
            # reset commands for new policy
            self.reset_policy(self.network)

        # Go-to-Init
        if msg.buttons[3] == 1 and not self.cp.status_panel.gti_click:  # (Y)
            self.go_to_init = not self.go_to_init

        # Update control panel
        self.cp.commands_panel.lin_vel_x = self.lin_vel_x
        self.cp.commands_panel.lin_vel_y = self.lin_vel_y
        self.cp.commands_panel.heading = self.heading
        self.cp.commands_panel.foot_height = self.special
        self.cp.commands_panel.x_click = msg.axes[1]
        self.cp.commands_panel.y_click = msg.axes[6]
        self.cp.commands_panel.h_click = msg.axes[0]
        self.cp.commands_panel.fh_click = msg.axes[7]
        self.cp.policies_panel.active = self.network_index
        self.cp.policies_panel.selected = self.next_network_index
        self.cp.policies_panel.sp_click = msg.buttons[4] | msg.buttons[5]
        self.cp.status_panel.active_policy = self.network
        self.cp.status_panel.start_stop = self.bml_res
        self.cp.status_panel.go_to_init = self.go_to_init
        self.cp.status_panel.gti_click = msg.buttons[3]

        # Show console
        self.live.refresh()

        # Create new message
        cmd = MenteeCmd()
        # Header
        cmd.header.frame_id = str(self.frame_id)
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        # Data
        cmd.lin_vel_x = self.lin_vel_x
        cmd.lin_vel_y = self.lin_vel_y
        cmd.heading = self.heading
        cmd.foot_height = self.special

        self.pub.publish(cmd)
        self.frame_id += 1

    def change_network(self, name: str) -> TriggerListService.Response:
        self.net_req.name = [name]
        fut = self.brain_change_net.call_async(self.net_req)
        return fut.result()

    def set_net_loop(self, data) -> SetBool.Response:
        self.net_start.data = data
        future = self.brain_main_loop.call_async(self.net_start)
        return future.result()

    def get_brain_status(self, req: BrainStatus.Request) -> BrainStatus.Response:
        BrainStatus.Response = self.brain_get_status.call(req)
        self.network = BrainStatus.Response.network
        self.go_to_init = BrainStatus.Response.go_to_init

        return BrainStatus.Response

    def reset_policy(self, policy: str):
        self.commands_params_cfg = self.network_cfg[policy].command_dict

        # Ranges
        if not self.sin_net:
            self.range_lin_vel_x = self.commands_params_cfg.ranges.get(
                "lin_vel_x", [float("-inf"), float("inf")]
            )
            self.range_lin_vel_y = self.commands_params_cfg.ranges.get(
                "lin_vel_y", [float("-inf"), float("inf")]
            )
            self.range_heading = self.commands_params_cfg.ranges.get(
                "heading", [float("inf"), float("-inf")]
            )
        else:
            self.range_lin_vel_x = self.commands_params_cfg.ranges.get(
                "sin_amp", [float("-inf"), float("inf")]
            )
            self.range_lin_vel_y = self.commands_params_cfg.ranges.get(
                "sin_omega", [float("-inf"), float("inf")]
            )
            self.range_heading = self.commands_params_cfg.ranges.get(
                "dof_selector", [float("inf"), float("-inf")]
            )
        self.range_special = self.commands_params_cfg.ranges.get(
            "foot_height", [float("-inf"), float("inf")]
        )

        # Increment step
        if not self.sin_net:
            self.step_x = self.commands_params_cfg.increment.get("lin_vel_x", 0.0)
            self.step_y = self.commands_params_cfg.increment.get("lin_vel_y", 0.0)
            self.step_heading = self.commands_params_cfg.increment.get("heading", 0.0)
        else:
            self.step_x = self.commands_params_cfg.increment.get("sin_amp", 0.0)
            self.step_y = self.commands_params_cfg.increment.get("sin_omega", 0.0)
            self.step_heading = self.commands_params_cfg.increment.get("dof_selector", 0.0)
        self.step_special = self.commands_params_cfg.increment.get("foot_height", 0.0)

    def reset_dof_selector(self, policy: str):
        self.commands_params_cfg = self.network_cfg[policy].command_dict

        self.lin_vel_x = self.commands_params_cfg.default_value.get("sin_amp", 0.0)
        self.lin_vel_y = self.commands_params_cfg.default_value.get("sin_omega", 0.0)

        # Ranges
        self.range_lin_vel_x = self.commands_params_cfg.ranges.get(
            "sin_amp", [float("-inf"), float("inf")]
        )
        self.range_lin_vel_y = self.commands_params_cfg.ranges.get(
            "sin_omega", [float("-inf"), float("inf")]
        )

    def run(self):
        rclpy.spin(self.node)
        self.get_brain_status(self.stat)

    @staticmethod
    def wrap_to_pi(angles):
        angles %= 2 * np.pi
        angles -= 2 * np.pi * (angles > np.pi)
        return angles


def main():
    joystick = Joystick()
    joystick.run()


if __name__ == "__main__":
    main()