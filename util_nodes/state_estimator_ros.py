#!/usr/bin/env python
import argparse
import logging
import threading
import time

import numpy as np
import rclpy
import yaml
from candle_ros2.msg import (
    CandleJointState,
    ImpedanceCommand,
    KalmanParams,
    MotionCommand,
    SavgolParams,
)
from common.msg import MenteeCmd, RigidBodyState, StateEstimate, Vector3LL
from common.srv import BrainStatus, TopicSwitch
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped #, CoordSys
from hydra import compose, initialize
from nav_msgs.msg import Odometry
from omegaconf import OmegaConf
from rich.console import Console
from rich.live import Live
from sensor_msgs.msg import Imu, JointState, Joy
from menteebot.sensors.imus.datatypes import CoordSys
from std_srvs.srv import SetBool
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from .control_panel import ControlPanel
from .state_estimation_modules.state_estimator_ekf_async_npy import StateEstimatorEKF

logger = logging.getLogger("menteebot")


class StateEstimationROS:
    def __init__(self):
        self.debug = False
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument(
            "--yaml", default="../configs/default.yaml", help="Configuration file"
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
        self.config = cfg.networks.main.state_estim
        #sprint(f"self.config: {self.config}")
        meas_modules = self.init_measurements(self.config)
        self.state_estimator = StateEstimatorEKF(self.config, meas_modules)
        self.warmup_wait_sec = self.config["params"]["warmup_wait_sec"]
        self.init_by_visual_odom = (
            self.config.params.init_by_visual_odom
            and self.config["measurements"]["visual_odometry"]["is_on"]
        )
        self.state_initialized = False
        self.first_init = True
        self.start_estimator = False
        self.brain_main_loop = False
        self.imu_def_topic = (
            self.config.params.imu_default_topic
        )  # "/zed2i/zed_node/imu/data"  # "/imu/data"

        self.frame_id = 0
        # Init ROS2
        rclpy.init()

        # Create ROS node
        self.node = rclpy.create_node("state_estimation")

        # Create publisher with topic-name
        #self.pub = self.node.create_publisher(RigidBodyState, "/state_estimation/stater", 10)
        self.pub = self.node.create_publisher(Imu, "/state_estimation/stater", 10)

        self.net_loop_srv = self.node.create_service(SetBool, "topic_switch", self.topic_switch)
        self.restart_estim = self.node.create_service(SetBool, "restart", self.restart_cmd)

        # Subscribers
        self.imu_sub = self.node.create_subscription(Imu, self.imu_def_topic, self.imu_callback, 10)
       # print('a1')
        if self.config["measurements"]["visual_odometry"]["is_on"]:
            self.visual_odom_topic = self.config["measurements"]["visual_odometry"][
                "topic"
            ]  # "/zed2i/zed_node/odom"
            #self.visual_odom_topic = "/zed2i/zed_node/odom"
          #  print('a2')
            self.visual_odom_sub = self.node.create_subscription(
                #Odometry, self.visual_odom_topic, self.visual_odom_callback, 10
                PoseStamped, self.visual_odom_topic, self.visual_odom_callback, 10
            )
          #  print('a2-1')
       # print('a3')
        if (
            self.config["measurements"]["leg_odometry"]["is_on"]
            or self.config["measurements"]["imu_biases"]["is_on"]
        ):
            self.motors_sub = self.node.create_subscription(
                JointState, "/candle/joint_states", self.motors_callback, 10
            )  # check

        # self.contact_sub = self.node.create_subscription(Joy, "/??/data", self.contact_callback, 10)
        # or "/md80/joint_states"

        # Network loop service - inorder to start/stop the main loop
        # self.net_loop_srv = self.node.create_service(SetBool, "net_loop_st", self.set_net_loop)
        self.st_time = self.node.get_clock().now().to_msg().sec

        # Network loop service - inorder to start/stop the main loop
        # self.topic_srv = self.node.create_service(SetBool, "topic_switch", self.change_topic)
        # self.switch_topic = None
        # needs the brain running. No matter the state

        # self.brain_get_status = self.node.create_client(BrainStatus, "brain_status")
        # while not self.brain_get_status.wait_for_service(timeout_sec=1.0):
        #     logger.info('Service "Brain - brain_status" not available, waiting again...')
        # self.start_estimator = BrainStatus.Request()

        # self.brain_main_loop = self.node.create_client(SetBool, "topic_switch")
        # while not self.brain_main_loop.wait_for_service(timeout_sec=1.0):
        #     logger.info('Service "Brain - main_loop" not available, waiting again...')
        # self.topic_switch = SetBool.Request()

        # self.restart_estim = self.node.create_client(SetBool, "restart")
        # while not self.restart_estim.wait_for_service(timeout_sec=1.0):
        #     logger.info('Service "Brain - main_loop" not available, waiting again...')
        # self.restarter = SetBool.Request()
       # print('a4')

    def topic_switch(self, switch_topic: SetBool.Request, response: SetBool.Response):
        if switch_topic.data:
         #   print(f"in switch")
            self.imu_def_topic = self.config.params.imu_data
            self.set_lin_acc_mode(True)
            response.success = True
            response.message = self.config.params.imu_data
        else:
       #     print(f"in switch else")
            self.imu_def_topic = self.config.params.imu_none
            self.set_lin_acc_mode(False)
            response.success = True
            response.message = self.config.params.imu_none

        self.imu_sub = self.node.create_subscription(Imu, self.imu_def_topic, self.imu_callback, 10)
       # print(f"imu_def_topic: {self.imu_def_topic}")
        return response

    def restart_cmd(self, restart: SetBool.Request, response: SetBool.Response):
        self.reset()
        response.success = True
        response.message = "Estimation was reset"

      #  print(f"imu_def_topic: {self.imu_def_topic}")

        return response

    def switch(self, data) -> SetBool.Response:
        self.topic_switch.data = data

      #  print(f"data: {data}")

        future = self.brain_main_loop.call_async(self.topic_switch)
        return future.result()

    def get_brain_status(self, req: BrainStatus.Request) -> BrainStatus.Response:
        #print("boom")
        BrainStatus.Response = self.brain_get_status.call(req)
       # print(f"hi from status: {BrainStatus.Response.go_to_init}")
        self.go_to_init = BrainStatus.Response.go_to_init

        return BrainStatus.Response

    def change_topic(self, req: TopicSwitch.Request) -> TopicSwitch.Response:
        # TopicSwitch.Response = self.switch_topic.call_async(req)
        #print(f"TopicSwitch.R")
        TopicSwitch.Response = self.switch_topic(req)
        # print(f"TopicSwitch.Response: {fut.result()}")

        return TopicSwitch.Response

    def convert_stamp_to_sec(self, stamp):
        return np.array([stamp.nanosec / 1e9 + stamp.sec])

    # def convert_sec_to_stamp(self, time_sec):
    #     time_sec = time_sec * 1e9
    #     return np.array([stamp.nanosec / 1e9 + stamp.sec])

    def warmup_wait_passed(self):
        cur_time = self.node.get_clock().now().to_msg().sec
        return (cur_time - self.st_time) > self.warmup_wait_sec

    def publish_est_state(self, estimation, header):
        # to_pub = StateEstimate()
        # # Header
        # to_pub.header.frame_id = str(self.frame_id)
        # #est_time = self.convert_sec_to_stamp(estimation['time'])
        # est_time = self.node.get_clock().now().to_msg()
        #
        # to_pub.header.stamp = est_time

        # Data
        if self.config.params.publish_unbiased:
            lin_acc_est_local = estimation["root_lin_acceleration"]
            ang_vel_est_local = estimation["root_ang_vel"]
            lin_acc_est_lab = estimation["root_lin_acceleration_lab"]
            ang_vel_est_lab = estimation["root_ang_vel_lab"]
        else:
            lin_acc_est_local = estimation["root_lin_acceleration_biased"]
            ang_vel_est_local = estimation["root_ang_vel_biased"]
            lin_acc_est_lab = estimation["root_lin_acceleration_biased_lab"]
            ang_vel_est_lab = estimation["root_ang_vel_biased_lab"]

        if False:
            imu_state_msg = RigidBodyState()
            imu_state_msg.name.append("base_link")

            rot = Quaternion()
            rot.x = float(estimation["root_rot"][0])
            rot.y = float(estimation["root_rot"][1])
            rot.z = float(estimation["root_rot"][2])
            rot.w = float(estimation["root_rot"][3])
            imu_state_msg.rot.append(rot)

            ang_vel = Vector3LL()
            ang_vel.local.x = float(ang_vel_est_local[0])
            ang_vel.local.y = float(ang_vel_est_local[1])
            ang_vel.local.z = float(ang_vel_est_local[2])
            ang_vel.lab.x = float(ang_vel_est_lab[0])
            ang_vel.lab.y = float(ang_vel_est_lab[1])
            ang_vel.lab.z = float(ang_vel_est_lab[2])
            imu_state_msg.ang_vel.append(ang_vel)

            acc = Vector3LL()
            acc.local.x = float(lin_acc_est_local[0])
            acc.local.y = float(lin_acc_est_local[1])
            acc.local.z = float(lin_acc_est_local[2])
            acc.lab.x = float(lin_acc_est_lab[0])
            acc.lab.y = float(lin_acc_est_lab[1])
            acc.lab.z = float(lin_acc_est_lab[2])
            imu_state_msg.acc.append(acc)

            lin_vel = Vector3LL()
            lin_vel.local.x = float(estimation["root_vel"][0])
            lin_vel.local.y = float(estimation["root_vel"][1])
            lin_vel.local.z = float(estimation["root_vel"][2])
            lin_vel.lab.x = float(estimation["root_vel_lab"][0])
            lin_vel.lab.y = float(estimation["root_vel_lab"][1])
            lin_vel.lab.z = float(estimation["root_vel_lab"][2])
            imu_state_msg.lin_vel.append(lin_vel)

            pos = Vector3()
            pos.x = float(estimation["root_pos"][0])
            pos.y = float(estimation["root_pos"][1])
            pos.z = float(estimation["root_pos"][2])
            imu_state_msg.pos.append(pos)
        else:
            imu_state_msg = Imu()

            header = Header()
            curr_epoch_time = time.time_ns()
            sec = curr_epoch_time // (10 ** 9)
            header.stamp.sec = sec
            header.stamp.nanosec = curr_epoch_time - sec * (10 ** 9)
            header.frame_id = str(self.frame_id)
            imu_state_msg.header = header

            orientation = Quaternion()
            orientation.x = float(estimation["root_rot"][0])
            orientation.y = float(estimation["root_rot"][1])
            orientation.z = float(estimation["root_rot"][2])
            orientation.w = float(estimation["root_rot"][3])
            imu_state_msg.orientation = orientation

            ang_vel = Vector3()
            ang_vel.x = float(ang_vel_est_local[0])
            ang_vel.y = float(ang_vel_est_local[1])
            ang_vel.z = float(ang_vel_est_local[2])
            imu_state_msg.angular_velocity = ang_vel

            lin_acc = Vector3()
            lin_acc.x = float(lin_acc_est_lab[0])
            lin_acc.y = float(lin_acc_est_lab[1])
            lin_acc.z = float(lin_acc_est_lab[2])
            imu_state_msg.linear_acceleration = lin_acc




       # print('PUBLISH')

        self.pub.publish(imu_state_msg)
        self.frame_id += 1
        #print('PUBLISH - 2')
        return curr_epoch_time

    def imu_callback(self, msg: Imu):
        pre_epoch_time = time.time_ns()
        #print("in imu_callback")
        if not self.warmup_wait_passed():
            return

        times = self.convert_stamp_to_sec(msg.header.stamp)
        if not self.state_initialized:# and not self.init_by_visual_odom:
            root_rot_target = np.array(
                [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            )[
                np.newaxis, ...
            ]  # msg.orientation.['root_rot_target'] check
            init_dict = {"root_rot": root_rot_target, "times": times}
            self.state_estimator.init_env(init_dict, first_init=self.first_init)
            self.first_init = False
            if not self.init_by_visual_odom:
                self.state_initialized = True
        # elif not self.state_initialized and self.init_by_visual_odom:
        #     return
        else:
            # Control inputs from IMU
            root_ang_vel = np.array(
                [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            )[
                np.newaxis, ...
            ]  # ['root_ang_vel']
            root_acceleration = np.array(
                [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ]
            )[
                np.newaxis, ...
            ]  # msg['root_acceleration']
            root_rot_target = np.array(
                [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            )[
                np.newaxis, ...
            ]  # msg['root_rot_target']

            control_inputs = {
                "data": {
                    "root_ang_vel": root_ang_vel,
                    "root_lin_acc": root_acceleration,
                },
                "times": times,
            }
            self.state_estimator.add_control_inputs(control_inputs)

            if self.config["measurements"]["imu_biases"]["is_on"]:
                imu_biases_dict = {
                    "data": {
                        "root_ang_vel": root_ang_vel,
                        "root_lin_acc": root_acceleration,
                        "root_rot": root_rot_target,
                    },
                    "times": times,
                }
                self.state_estimator.measurements_modules["imu_biases"].add_imu_outputs(
                    imu_biases_dict
                )

            if self.config["measurements"]["root_rot"]["is_on"]:
                # Root orientation measurement from IMU
                root_rot_meas_dict = {
                    "data": {"root_rot_target": root_rot_target},
                    "times": times,
                }
                self.state_estimator.measurements_modules["root_rot"].add(root_rot_meas_dict)

            # xx = self.get_brain_status(self.start_estimator)
            # print(f"checking :   {xx}")
            #print('SHOULD PUBLISH - before')

            estimation, has_new_update = self.estimate()
            #print('SHOULD PUBLISH')
            # if has_new_update:
            post_epoch_time = self.publish_est_state(estimation)
            time_diff = post_epoch_time - pre_epoch_time

            #print(time_diff / (10**9))
            # print(f"estimation: {estimation}")

    # def root_pos_z_callback(self, msg):
    #     if (not self.config['measurements']['root_pos_z']['is_on']) or (not self.config['measurements']['root_vel_world_z']['is_on']):
    #         return
    #     if not self.state_initialized:
    #         return
    #     time = msg['times']
    #     root_pos_z = msg['root_pos_z']
    #     root_pos_z_dict = {
    #         'data': {'root_pos_z': root_pos_z},
    #         'times': time
    #     }
    #     if self.config['measurements']['root_pos_z']['is_on']:
    #         self.state_estimator.measurements_modules['root_pos_z'].add(root_pos_z_dict)
    #     if self.config['measurements']['root_vel_world_z']['is_on']:
    #         self.state_estimator.measurements_modules['root_vel_world_z'].add(root_pos_z_dict)

    def motors_callback(self, msg: JointState):
        if (not self.config["measurements"]["leg_odometry"]["is_on"]) and (
            not self.config["measurements"]["imu_biases"]["is_on"]
        ):
            return
        if not self.state_initialized:
            return

        time = self.convert_stamp_to_sec(msg.header.stamp)
        motors_pos = np.asarray(msg.position)[np.newaxis, ...]
        motors_vel = np.asarray(msg.velocity)[np.newaxis, ...]

        # feet_pos_base = msg['feet_pos_base']
        motors_pos_dict = {
            "data": {
                "motors_pos": motors_pos,
                "motors_vel": motors_vel
                # 'feet_pos_base': feet_pos_base,
                #'root_rot': msg['root_rot'], #debug
            },
            "times": time,
        }
        if self.debug:
            motors_pos_dict["data"]["root_rot"] = (msg["root_rot"],)  # debug

        if self.config["measurements"]["leg_odometry"]["is_on"]:
            self.state_estimator.measurements_modules["leg_odometry"].add_motors_pos(
                motors_pos_dict
            )
        if self.config["measurements"]["imu_biases"]["is_on"]:
            self.state_estimator.measurements_modules["imu_biases"].add_motors_pos(motors_pos_dict)
            # self.state_estimator.measurements_modules["imu_biases"].add_motors_vel(motors_pos_dict)

    #def visual_odom_callback(self, msg: Odometry):
    def visual_odom_callback(self, msg: PoseStamped):

       # print('a')
        if not self.config["measurements"]["visual_odometry"]["is_on"]:
            return
       # print('b')
        if not self.warmup_wait_passed():
            return
      #  print('c')

        time = self.convert_stamp_to_sec(msg.header.stamp)
        cam_pos_ros = msg.pose.position #msg.pose.pose.position
        cam_pos = [cam_pos_ros.x, cam_pos_ros.y, cam_pos_ros.z]
        cam_pos = np.asarray(cam_pos)[np.newaxis, ...]

        cam_rot_ros = msg.pose.orientation #msg.pose.pose.orientation
        cam_rot_ros = [cam_rot_ros.x, cam_rot_ros.y, cam_rot_ros.z, cam_rot_ros.w]
        cam_rot = np.asarray(cam_rot_ros)[np.newaxis, ...]

        if not self.state_initialized and not self.init_by_visual_odom:
            return
        if not self.state_initialized and self.init_by_visual_odom:
            # root_pos = self.state_estimator.measurements_modules["visual_odometry"].cam_root_pos_to_imu_root_pos(
            #     cam_pos
            # )
            if self.state_estimator.measurements_modules["visual_odometry"].config['align_cam_quat_to_imu']:
                if self.first_init:
                    return
                imu_rot = self.state_estimator.get_state()['root_rot']
            else:
                imu_rot = None #self.state_estimator.get_state()
            root_rot_target = self.state_estimator.measurements_modules["visual_odometry"].convert_cam_rot_to_imu_rot(cam_rot, imu_rot)
            root_pos = self.state_estimator.measurements_modules["visual_odometry"].convert_cam_pos_to_imu_pos(cam_pos, root_rot_target)
            init_dict = {"root_rot": root_rot_target, "root_pos": root_pos, "times": time}
            self.state_estimator.init_env(init_dict)
            self.first_init = False
            self.state_initialized = True
        else:
            visual_odom_dict = {
                "data": {"cam_pos": cam_pos, "cam_rot": cam_rot},
                "times": time,
            }
            self.state_estimator.measurements_modules["visual_odometry"].add(visual_odom_dict)

    def contact_callback(self, msg):
        if not self.config["measurements"]["leg_odometry"]["is_on"]:
            return
        if not self.state_initialized:
            return
        time = self.convert_stamp_to_sec(msg.header.stamp)
        feet_contact_force = msg["feet_contact_force"]
        feet_contact_force_dict = {
            "data": {
                "feet_contact_force": feet_contact_force,
                #'feet_vel': msg['feet_vel'],
                #'root_vel': msg['root_vel'],  # debug
            },
            "times": time,
        }
        if self.debug:
            feet_contact_force_dict["data"]["feet_vel"] = msg["feet_vel"]
            feet_contact_force_dict["data"]["root_vel"] = msg["root_vel"]

        self.state_estimator.measurements_modules["leg_odometry"].add_feet_contact_force(
            feet_contact_force_dict
        )

    def get_last_estimated_state(self):
        estimated_state = self.state_estimator.get_state()
        return estimated_state

    def estimate(self):
        if not self.state_initialized:
            return
        new_update = self.state_estimator.estimate_async()
        estimated_state = self.state_estimator.get_state()
        for key in estimated_state.keys():
            if type(estimated_state[key]) == np.ndarray:
                estimated_state[key] = estimated_state[key].squeeze()

        # if new_update:
        #     estimated_state = self.state_estimator.get_state()
        # else:
        #     return None
        #
        return estimated_state, new_update

    def reset(self):
        self.state_estimator.set_reset_buffer()
        self.state_estimator.reset()
        self.state_initialized = False
        self.first_init = True

    def set_lin_acc_mode(self, zero_at_rest):
        self.config["lin_acc_zero_at_rest"] = zero_at_rest
        self.state_estimator.set_lin_acc_mode(zero_at_rest)

    def init_measurements(self, config):
        meas_config = config["measurements"]
        meas_modules = {}
        # meas_config[
        #     "lin_acc_zero_at_rest"
        # ] = config.params.lin_acc_zero_at_rest  # ['params']['lin_acc_zero_at_rest']

        for meas_name in meas_config.keys():
            meas_config[meas_name]["lin_acc_zero_at_rest"] = config.params.lin_acc_zero_at_rest

        if meas_config["leg_odometry"]["is_on"]:
            meas_config["leg_odometry"]["debug"] = self.debug
            from .state_estimation_modules.leg_odometry_module_npy import (
                LegOdometrytModule,
            )

            leg_odometry_module = LegOdometrytModule(meas_config["leg_odometry"])
            meas_modules["leg_odometry"] = leg_odometry_module

        # if meas_config['root_vel_world_z']['is_on']:
        #     from .root_vel_world_z_module_npy import RootVelWorldZModule
        #     root_vel_world_z_module = RootVelWorldZModule(meas_config['root_vel_world_z'])
        #     meas_modules['root_vel_world_z'] = root_vel_world_z_module

        if meas_config["root_pos_z"]["is_on"]:
            from .state_estimation_modules.root_pos_z_module_npy import RootPosZModule

            root_pos_z_module = RootPosZModule(meas_config["root_pos_z"])
            meas_modules["root_pos_z"] = root_pos_z_module

        if meas_config["root_rot"]["is_on"]:
            from .state_estimation_modules.root_rot_module_npy import RootRotModule

            root_rot_module = RootRotModule(meas_config["root_rot"])
            meas_modules["root_rot"] = root_rot_module

        if meas_config["imu_biases"]["is_on"]:
            from .state_estimation_modules.imu_biases_module_npy import ImuBiasesModule

            # from .state_estimation_modules.imu_biases_module_motors_vel_npy import ImuBiasesModule
            imu_biases_module = ImuBiasesModule(meas_config["imu_biases"])
            meas_modules["imu_biases"] = imu_biases_module

        if meas_config["visual_odometry"]["is_on"]:
            meas_config["visual_odometry"]["debug"] = self.debug
            from .state_estimation_modules.visual_odom_module_npy import (
                VisualOdomModule,
            )

            visual_odometry_module = VisualOdomModule(meas_config["visual_odometry"])
            meas_modules["visual_odometry"] = visual_odometry_module

        # if meas_config['root_pos']['is_on']:
        #     from util_nodes.state_estimation_modules.root_pos_module_npy import RootPosModule
        #     root_pos_module = RootPosModule(meas_config['root_pos'])
        #     meas_modules['root_pos'] = root_pos_module
        #     if 'imu_biases' in meas_config.keys() and meas_config['imu_biases']['is_on'] and meas_config['imu_biases']['reset_first_pos']:
        #         meas_modules['imu_biases'].add_root_pos_module(meas_modules['root_pos'])

        return meas_modules

    def run(self):
        rclpy.spin(self.node)
        # self.get_brain_status(self.start_estimator)


def main():
    state = StateEstimationROS()  # "./state_estimation_modules/state_estim.yaml")
    state.run()


#
#
# if __name__ == "__main__":
#     main()
