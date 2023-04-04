from dataclasses import dataclass, field
from typing import List



@dataclass
class DoFStateData:
    msg_seq: int = 0
    msg_stamp: float = 0.0
    pos: float = 0.0
    vel: float = 0.0
    savgol_vel: float = 0.0
    kalman_vel: float = 0.0
    torque: float = 0.0
    handler_name: str = ""

@dataclass
class CommandData:
    msg_frame_id: int = 0
    msg_stamp: float = 0.0
    data: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

@dataclass
class GripperData:
    msg_frame_id: int = 0
    msg_stamp: float = 0.0
    desired_position: float = 0.0
    actual_position: float = 0.0
    mlx_x: float = 0.0
    mlx_y: float = 0.0
    mlx_xz: float = 0.0
