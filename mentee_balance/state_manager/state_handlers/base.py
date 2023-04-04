from abc import abstractmethod
from typing import Dict, List, Optional, Tuple

from rclpy.node import Node as RosNode


class BaseManager:
    def __init__(self, cfg, ros_node: Optional[RosNode] = None):
        self.cfg = cfg
        self.ros_node = ros_node

    @abstractmethod
    def get_states(
        self, names: Optional[List[str]] = None, states: Optional[List[str]] = None, **kwargs
    ) -> Tuple[Dict[str, List[float]], List]:
        """Get the current state"""

    @abstractmethod
    def publish_state(self, action_dict: Dict[str, float], req_id: int):
        """Publish state"""
