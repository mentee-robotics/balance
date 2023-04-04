from dataclasses import dataclass

@dataclass
class MotorData:
    msg_frame_id: int = 0
    msg_stamp: float = 0.0
    position: float = 0.0
    velocity: float = 0.0
    savgol_vel: float = 0.0
    kalman_vel: float = 0.0
    torque: float = 0.0
