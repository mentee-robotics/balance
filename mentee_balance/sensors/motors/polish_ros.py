import logging
import time
from collections import defaultdict

import mentee_messages.srv as candle_services
#from candle_ros2.srv import AddMd80s, GenericMd80Msg, SetModeMd80s


from mentee_messages.msg import (
    CandleJointState,
    ImpedanceCommand,
    MotionCommand,
)
from mentee_messages.srv import AddMd80s, GenericMd80Msg, SetModeMd80s

import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import QoSProfile
from rclpy.task import Future
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from .base import BaseMotor
from .datatypes import MotorData
import sys
sys.path.append("../..")

from utils.recorder import Recorder


logger = logging.getLogger("menteebot")


class PolishROS(BaseMotor):
    def get_response_callback(self, prefix_text: str):
        """
        create a callback function for when ros service is done
        Args:
            prefix_text: Text to print when service is done
        Returns: Callback function

        """

        def response_callback(response: Future):
            logger.info(f"{prefix_text} {self.name} {response.result()} ")

        return response_callback

    def call_service(self, client, request, text=""):
        """
        This function calls a ros service.
        Args:
            client: the service client
            request: The request to send
            text: Optional text to print when service is done

        Returns: None

        """
        # call service and create callback to print text when the service finished
        #future = client.call_async(request)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.ros_node, future)
        future.add_done_callback(self.get_response_callback(text))
        return future

    def init(self):
        logger.info("*****  INIT START *********")
        self.data = defaultdict(MotorData)
        logger.info("*****  INIT START *********2")
        # Motors subscribers
        self.ros_node.create_subscription(
            CandleJointState, "/md80/joint_states", self.get_data, QoSProfile(depth=10)
        )
        logger.info("*****  INIT START *********3")
        # Motors publishers
        self.publisher = self.ros_node.create_publisher(MotionCommand, self.cfg.topics.pub, 5)
        self.publish_update_md80_impedance = self.ros_node.create_publisher(
            ImpedanceCommand, "/md80/impedance_command", 10  #dimaTODO: how to migrate this to yaml?
        )
        logger.info("*****  INIT START *********4")
        # Motor services
        self.setmodemd80_srv = self.ros_node.create_client(SetModeMd80s, "/candle_ros2_node/set_mode_md80s")
        self.setmodemd80_srv.wait_for_service()
        self.addmd80s_srv = self.ros_node.create_client(AddMd80s, "/candle_ros2_node/add_md80s")
        self.addmd80s_srv.wait_for_service()
        self.enablemd80_srv = self.ros_node.create_client(GenericMd80Msg, "/candle_ros2_node/enable_md80s")
        self.enablemd80_srv.wait_for_service()
        logger.info("*****  INIT START *********5")

        logger.info("*****  INIT START *********6")
        self.recorder = Recorder(
            f"{self.name}.csv",
            self.ros_node,
            fields_names=["drive_ids", "frame_ids", "msg_stamp", "motor_stamps", "curr_time"],
        )
        logger.info("*****  INIT END *********")

    def change_to_impedance_mode(self):
        """
        Changes motor control to impedance mode and set the initial parameters for this control
        Returns:
        """
        # create empty request for mode change
        mode_request = SetModeMd80s.Request()
        impedance_kp_kd_command = ImpedanceCommand()

        # Update the request with the motor ids and kp,kd,max_torque params
        for motor_id in self.motor_id2rigid_body:
            mode_request.drive_ids.append(motor_id)
            mode_request.mode.append("IMPEDANCE")
            impedance_kp_kd_command.drive_ids.append(motor_id)
            impedance_kp_kd_command.kp.append(self.init_kp[motor_id])
            impedance_kp_kd_command.kd.append(self.init_kd[motor_id])
            impedance_kp_kd_command.max_output.append(self.max_torque[motor_id])

        # Change mode
        self.call_service(self.setmodemd80_srv, mode_request, "Change mode")

        # Update kp,kd
        logger.info(f"UPDATING IMPEDANCE COMMAND")
        self.publish_update_md80_impedance.publish(impedance_kp_kd_command)
        logger.info(f"UPDATING IMPEDANCE COMMAND DONE")

    def add_motors(self):
        add_request = AddMd80s.Request()
        for motor_id in self.motor_id2rigid_body:
            add_request.drive_ids.append(motor_id)
            add_request.watchdog_kp.append(self.watchdog_params[motor_id]["kp"])
            add_request.watchdog_kd.append(self.watchdog_params[motor_id]["kd"])
            add_request.max_position.append(self.watchdog_params[motor_id]["max_position"])
            add_request.min_position.append(self.watchdog_params[motor_id]["min_position"])
            add_request.pos_percent_wanted.append(
                self.watchdog_params[motor_id]["pos_percent_wanted"]
            )
            add_request.soft_limit.append(self.watchdog_params[motor_id]["soft_limit"])
            add_request.torque_offset.append(self.watchdog_params[motor_id]["torque_offset"])

        self.call_service(self.addmd80s_srv, add_request, "Added")

    def boot(self):
        logger.info("*****  STARTING BOOT *********")
        self.init()

        # Add all motors_handlers

        self.add_motors()
        logger.info("***** Added MOTORS *********")

        self.change_to_impedance_mode()
        logger.info("*****  CHANGED TO IMPEDANCE MODE *********")


        # Set mode
        enable_request = GenericMd80Msg.Request()
        enable_request.drive_ids = list(self.motor_id2rigid_body.keys())

        logger.info("*****  ENABLING MOTORS *********")
        # Enable motors_handlers
        self.call_service(self.enablemd80_srv, enable_request, "Enabled motors_handlers")
      #  self.call_service(self.enableAllMotors_srv, enable_request, "Enabled motors_handlers")

        self.enable = True
        logger.info("*****  BOOT END *********")


    def go_to_init(self):
        """
        Send command to motors to move to init position
        """
        logger.info("*****  GOING TO INIT *********")
        init_command = MotionCommand()

        # fill init_command's lists with the motors and there commands
        for motor_id, dof_name in self.motor_id2rigid_body.items():
            # frame id -1, so we can separate it from the ones given by the policy
            init_command.header.frame_id = str(-1)
            init_command.drive_ids.append(motor_id)
            init_command.target_position.append(self.init_positions[dof_name])
            init_command.target_velocity.append(0.0)
            init_command.target_torque.append(0.0)

            # We use init kp kd that is different from the default for policy
            init_command.kp.append(self.init_kp[motor_id])
            init_command.kd.append(self.init_kd[motor_id])

            logger.info(f"*****  GOING TO {motor_id} ****{dof_name}*****{self.init_positions[dof_name]}")

        logger.info(f"Applying INIT COMMAND  {init_command}")
        self.publisher.publish(init_command)


    def get_data(self, msg: CandleJointState):
        """
        Get the current data from the robot
        Args:
            msg:

        Returns:
        """
        curr_time = time.time()
        msg_stamp = msg.header.stamp.sec + (1e-9 * msg.header.stamp.nanosec)

        # Log
        self.recorder.record_row(
            {
                "drive_ids": str(msg.name),
                "frame_ids": str(msg.frame_ids),
                "msg_stamp": str(msg_stamp),
                "motor_stamps": str(msg.time_stamps),
                "curr_time": str(curr_time),
            }
        )

        # Update data for each motor and then return it
        for i, name in enumerate(msg.name):  # i: int, name: str
        #    logger.info(f"i = {i} mame = {name} msg.name = {msg.name} ")
            self.data[name].msg_frame_id = msg.frame_ids[i]
            self.data[name].msg_stamp = msg.time_stamps[i]
            self.data[name].position = msg.position[i]
            self.data[name].velocity = msg.velocity[i]
            self.data[name].torque = msg.effort[i]

            yield name, self.data[name]

    def _get_single_dof_data(self, motor_id, state):
        return getattr(self.data[motor_id], state)

    def apply(self, req_id: int, motors_command):
        motors_command.header.frame_id = str(req_id)
        motors_command.header.stamp = self.ros_node.get_clock().now().to_msg()

        logger.info(f"Applying something {motors_command}")
        self.publisher.publish(motors_command)
