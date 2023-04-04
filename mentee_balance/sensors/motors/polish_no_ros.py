import argparse
import logging
import math
import time, sys

import menteepybind.pyCandle as pyCandle
from hydra import compose, initialize
from omegaconf import OmegaConf
from rich.console import Console
import sys
sys.path.append("../..")
from utils.recorder import Recorder
from .base import BaseMotor

console = Console()
logger = logging.getLogger("menteebot")


class PolishNoROS(BaseMotor):
    def change_to_impedance_mode(self):
        # Now we shall loop over all found drives to change control mode and enable them one by one
        self.candles_handler.setModeMd80(self.motor_ids, pyCandle.Md80Mode_E.IMPEDANCE)
        motors_impedance_params = dict()
        for motor_id in self.motor_ids:
            motors_impedance_params[motor_id] = dict()
            motors_impedance_params[motor_id]["kp"] = self.init_kp[motor_id]
            motors_impedance_params[motor_id]["kd"] = self.init_kd[motor_id]
            motors_impedance_params[motor_id]["max_torque"] = self.max_torque[motor_id]
        self.candles_handler.setImpedanceParameters(motors_impedance_params)

    def __del__(self):
        self.candles_handler.disableAllMotors()

    def add_motors(self):
        motors_to_add = dict()
        for motor_id in self.motor_ids:
            motors_to_add[motor_id] = self.watchdog_params[motor_id]
        self.candles_handler.addMd80s(motors_to_add)

    def set_kalman_filter(self):
        kalman_process_noise_config = {}
        kalman_measurement_noise_config = {}
        kalman_initial_state_noise_config = {}
        for motor_id, kalman_params in self.kalman_filter_params.items():
            kalman_process_noise_config[motor_id] = [
                kalman_params["transition_factor"] * kalman_params["pos_variance"],
                0.0,
                0.0,
                kalman_params["transition_factor"] * kalman_params["vel_variance"],
            ]
            kalman_measurement_noise_config[motor_id] = [
                kalman_params["pos_variance"],
                0.0,
                0.0,
                kalman_params["vel_variance"],
            ]
            kalman_initial_state_noise_config[motor_id] = [
                kalman_params["pos_variance"],
                0.0,
                0.0,
                kalman_params["vel_variance"],
            ]
        self.candles_handler.setKalmanFilter(
            kalman_process_noise_config,
            kalman_measurement_noise_config,
            kalman_initial_state_noise_config,
            500,
        )

    def set_pid_params(self):
        motors_pid_params = dict()
        for motor_id, motor_name in self.motor_id2rigid_body.items():
            motors_pid_params[motor_id] = dict()
            motors_pid_params[motor_id]["high_p_gain"] = self.pid_params["high_pid_kp"][motor_name]
            motors_pid_params[motor_id]["high_d_gain"] = self.pid_params["high_pid_kd"][motor_name]
            motors_pid_params[motor_id]["high_i_gain"] = self.pid_params["high_pid_ki"][motor_name]
            motors_pid_params[motor_id]["high_max_agg"] = self.pid_params["high_pid_max_agg_error"][
                motor_name
            ]
            motors_pid_params[motor_id]["high_limit_scale"] = self.pid_params[
                "high_pid_actions_limit_scale"
            ][motor_name]
            motors_pid_params[motor_id]["agg_window"] = -1.0
        self.candles_handler.setPIDParams(motors_pid_params)

    def boot(self):
        baud = pyCandle.CANdleBaudrate_E.CAN_BAUD_2M
        bus = pyCandle.BusType_E.SPI
        # if self.candle_params.baudrate == "8M":
        #     baud = pyCandle.CANdleBaudrate_E.CAN_BAUD_8M
        # if self.candle_params.bus_type == "SPI":
        #     bus = pyCandle.BusType_E.SPI

        # self.candles_handler = pyCandle.CandleHandler(True)
        self.candles_handler = pyCandle.CandleHandler(baud, bus, "", True)
        self.motor_ids = list(self.motor_id2rigid_body.keys())

        self.recorder = Recorder(
            f"{self.name}.csv",
            None,
            fields_names=["drive_ids", "frame_ids", "motor_stamps", "curr_time"],
        )

        # Add all motors to candles
        self.add_motors()
        self.change_to_impedance_mode()

        # Print all motors' status
        self.candles_handler.getExtendedDiagnostics()

        # enable all the motors
        self.candles_handler.enableAllMotors()

        # TODO: need to decide if we want to find a
        #  way to set this from the outside or remove it all together
        self.enable = True

    def go_to_init(self):
        """
        Send command to motors to move to init position
        """
        motors_command = dict()

        # Fill the command dict with dictionaries of targets for each motor
        for dof_name, can_id in self.rigid_body2motor_id.items():
            # for pybind we use a dictionary for each motor to set the command
            motors_command[can_id] = dict()
            motors_command[can_id]["position"] = self.init_positions[dof_name]
            motors_command[can_id]["velocity"] = 0.0
            motors_command[can_id]["torque"] = 0.0
            motors_command[can_id]["kp"] = self.init_kp[can_id]
            motors_command[can_id]["kd"] = self.init_kd[can_id]

        self.apply(-1, motors_command)

    def smoother_init(self):

        #motors_status = self.candles_handler.getMotorsData(self.motor_ids)
        t = 0.0
        i = 0
        move_res = 100
        dt = 1 / 6000
        SIN_AMP = 0.6

        while i < move_res:  # self.check_location():
            motors_command = dict()

            for dof_name, can_id in self.rigid_body2motor_id.items():
                motors_command[can_id] = dict()


                if can_id == 105:
                    #traj_val = SIN_AMP * math.sin(0.5 * 2 * math.pi * t)
                    step = abs(self.init_positions[dof_name] / move_res)
                    if step == 0:
                        step = 0.001


                    target = self._get_single_dof_data(can_id, "position")["position"]
                    logger.info(f"target: {target}, type: {type(target)}")
                    logger.info(f"self.init_positions[dof_name]: {self.init_positions[dof_name]}, type: {type(self.init_positions[dof_name])}")

                    if target < self.init_positions[dof_name]:
                        target += step

                    if target > self.init_positions[dof_name]:
                        target -= step
                    motors_command[can_id]["position"] = target
                else:
                    motors_command[can_id]["position"] = self.init_positions[dof_name]

                motors_command[can_id]["velocity"] = 0.0
                motors_command[can_id]["torque"] = 0.0
                motors_command[can_id]["kp"] = self.init_kp[can_id]
                motors_command[can_id]["kd"] = self.init_kd[can_id]

            t = t + dt

            #self.apply(-1, motors_command)
            time.sleep(1)

            i += 1

        # while self.check_location():
        #     motors_command = dict()
        #     for dof_name, can_id in self.rigid_body2motor_id.items():
        #         traj_val = SIN_AMP * math.sin(0.5 * 2 * math.pi * t)
        #
        #         motors_command[can_id] = dict()
        #         motors_command[can_id]["position"] = self._get_single_dof_data(can_id, "position") + dt
        #         #motors_command[can_id]["position"] = traj_val
        #         motors_command[can_id]["velocity"] = 0.0
        #         motors_command[can_id]["torque"] = 0.0
        #         motors_command[can_id]["kp"] = self.init_kp[can_id]
        #         motors_command[can_id]["kd"] = self.init_kd[can_id]
        #
        #     t = t + dt
        #     self.apply(-1, motors_command)

    def check_location(self):
        check_list = []
        for dof_name, can_id in self.rigid_body2motor_id.items():
            curr_loc = self._get_single_dof_data(can_id, "position")["position"]
            logger.info(f"can_id: {can_id}, curr_loc: {curr_loc}")
            if abs(curr_loc - self.init_positions[dof_name]) > 0.0001:
                check_list.append(False)
            else:
                check_list.append(True)
        return all(check_list)

    def get_data(self):
        """
        Get the current data from the robot.
        """

        # Get all motors current data from the candle handler
        motors_status = self.candles_handler.getMotorsData(self.motor_ids)

        # These vars are to save the data for recording
        curr_time = time.time()
        drive_ids = []
        frame_ids = []
        time_stamps = []

        # generate the data for each motor.
        for motor_id, motor_data in motors_status.items():  # motor_id: int, motor_data: dict
            drive_ids.append(motor_id)
            frame_ids.append(motor_data["seq"])
            time_stamps.append(motor_data["time"])
            yield self.motor_id2rigid_body[motor_id], motor_data

        # Log
        self.recorder.record_row(
            {
                "drive_ids": str(drive_ids),
                "frame_ids": str(frame_ids),
                "motor_stamps": str(time_stamps),
                "curr_time": str(curr_time),
            }
        )

    def _get_single_dof_data(self, motor_id, state):
        return self.candles_handler.getMotorsData([motor_id])[motor_id][state]

    def apply(self, req_id, motors_command):
        logger.info(f"apply: {motors_command}")
        self.candles_handler.sendMotorCommand(req_id, motors_command)
