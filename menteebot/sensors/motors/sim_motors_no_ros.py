import logging
import time

from rich.console import Console

from ...utils.recorder import Recorder
from .base import BaseMotor

console = Console()
logger = logging.getLogger("menteebot")


class SimMotorNoROS(BaseMotor):
    def change_to_impedance_mode(self):
        logger.info(f"changing {self.motor_ids} to impedance mode")

    def add_motors(self):
        logger.info(f"adding motoros {self.motor_ids}")

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
        logger.info(
            f"kalman_filter \nparams kalman_proccess_noise_config: {kalman_process_noise_config}\n kalman_measurment_noise_config {kalman_measurement_noise_config}\nkalman_initial_pos_noise_config: {kalman_initial_state_noise_config}"
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
        logger.info(f"pid params are: {motors_pid_params}")

    def boot(self):
        self.motor_ids = list(self.motor_id2rigid_body.keys())
        self.seq = 0
        self.motor_data = {
            "temperature": 0.0,
            "position": 0.1,
            "velocity": 0.0,
            "torque": 0.3,
            "savgol_vel": 0.0,
            "kalman_vel": 0.0,
            "our_velocity": 0.0,
            "time": time.time(),
            "seq": self.seq,
        }

        self.recorder = Recorder(
            f"{self.name}.csv",
            None,
            fields_names=["drive_ids", "frame_ids", "motor_stamps", "curr_time"],
        )

        # Add all motors to candles
        self.add_motors()

        # if self.savgol_coeffs:
        #     logger.info(f"savgol is: {self.savgol_coeffs}")
        #
        # if self.kalman_filter_params:
        #     self.set_kalman_filter()

        if self.pid_params:
            self.set_pid_params()

        self.change_to_impedance_mode()

        # enable all the motors

        # TODO: need to decide if we want to find a
        #  way to set this from the outside or remove it all together
        self.enable = True

    def go_to_init(self):
        """
        Send command to motors to move to init position
        """
        logger.info("went to init")

    def get_data(self):
        """
        Get the current data from the robot.
        """

        # These vars are to save the data for recording
        curr_time = time.time()
        drive_ids = []
        frame_ids = []
        time_stamps = []
        self.seq += 1
        self.motor_data["time"] = time.time()
        self.motor_data["seq"] = self.seq
        # generate the data for each motor.
        for motor_id in self.motor_ids:  # motor_id: int, motor_data: dict
            drive_ids.append(motor_id)
            frame_ids.append(self.motor_data["seq"])
            time_stamps.append(self.motor_data["time"])
            yield self.motor_id2rigid_body[motor_id], self.motor_data

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
        return self.motor_data

    def apply(self, req_id, motors_command):
        logger.info(f"apply: {motors_command}")
