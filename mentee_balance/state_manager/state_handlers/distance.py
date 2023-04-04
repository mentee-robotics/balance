from collections import defaultdict
from typing import Callable, Dict, List, Optional, Tuple, Type

from ...sensors import distance
from ...sensors.distance import BaseDistance, DistanceData
from .base import BaseManager


class DistanceStateManager(BaseManager):
    def __init__(self, cfg, ros_node):
        super(DistanceStateManager, self).__init__(cfg, ros_node)

        # Dictionary of all distance sensors data
        self.sensors_data = defaultdict(DistanceData)

        # Dictionary of all IMUs
        self.distance_sensors = dict()

        # Init IMUs
        for name, sensor_cfg in self.cfg.sensors.items():
            # Get sensor class
            distance_sensor_cls: Type[BaseDistance] = getattr(distance, sensor_cfg.cls)
            # Init distance sensor class
            sensor_handler: BaseDistance = distance_sensor_cls(name, sensor_cfg, ros_node)
            # Save imu class in dictionary
            self.distance_sensors[name] = sensor_handler

    def update(self):
        for name, handler in self.distance_sensors.items():
            self.sensors_data[name] = handler.get_data()

    def get_states(
        self, names: Optional[List[str]] = None, states: Optional[List[str]] = None, **kwargs
    ) -> Tuple[Dict[str, float], List]:
        # Init dict
        distance_states = defaultdict(float)
        seq_stamps = []

        # If no names are provided, get all the names in self.distance_sensors
        if names is None:
            names = list(self.distance_sensors.keys())

        # if states is None:
        #     states = ["pos", "pos", "lin_vel", "ang_vel", "acc"]

        # Retrieve data
        for distance_sensor_name in names:
            if distance_sensor_name not in self.distance_sensors:
                raise Exception(f"Sensor {distance_sensor_name} does not exists.")
            data = self.sensors_data[distance_sensor_name]
            seq_stamps.append((data.msg_frame_id, data.msg_stamp))
            distance_states[distance_sensor_name] = data.distance

        return distance_states, seq_stamps
