from dataclasses import dataclass


@dataclass
class CommandTwistData:
    msg_frame_id: str = 0
    msg_stamp: float = 0.0
    linear_vel_x: float = 0.0
    linear_vel_y: float = 0.0
    linear_vel_z: float = 0.0
    angular_vel_x: float = 0.0
    angular_vel_y: float = 0.0
    angular_vel_z: float = 0.0


@dataclass
class CommandData:
    msg_frame_id: str = 0
    msg_stamp: float = 0.0
    lin_vel_x: float = 0.0
    lin_vel_y: float = 0.0
    heading: float = 0.0
    foot_height: float = 0.0
