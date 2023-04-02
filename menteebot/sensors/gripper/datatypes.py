from dataclasses import dataclass

@dataclass
class GripperData:
    msg_frame_id: int = 0
    msg_stamp: float = 0.0
    desired_position: float = 0.0
    actual_position: float = 0.0
    mlx_x: float = 0.0
    mlx_y: float = 0.0
    mlx_z: float = 0.0
