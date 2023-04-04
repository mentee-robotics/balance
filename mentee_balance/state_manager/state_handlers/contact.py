from collections import defaultdict
from typing import Callable, Dict, List, Optional, Tuple, Type

from ...sensors import contact
from ...sensors.contact import BaseContact, ContactData
from .base import BaseManager


class ContactStateManager(BaseManager):
    def __init__(self, cfg, ros_node):
        super(ContactStateManager, self).__init__(cfg, ros_node)

        # Dictionary of all distance sensors data
        self.sensors_data = defaultdict(ContactData)

        # Dictionary of all IMUs
        self.contact_sensors = dict()

        # Init IMUs
        for name, sensor_cfg in self.cfg.sensors.items():
            # Get sensor class
            contact_sensor_cls: Type[BaseContact] = getattr(contact, sensor_cfg.cls)
            # Init distance sensor class
            sensor_handler: BaseContact = contact_sensor_cls(name, sensor_cfg, ros_node)
            # Save imu class in dictionary
            self.contact_sensors[name] = sensor_handler

    def update(self):
        for name, handler in self.contact_sensors.items():
            self.sensors_data[name] = handler.get_data()

    def get_states(
        self, names: Optional[List[str]] = None, states: Optional[List[str]] = None, **kwargs
    ) -> Tuple[Dict[str, List[float]], List]:
        # Init dict
        contact_states = defaultdict(List[float])
        seq_stamps = []

        # If no names are provided, get all the names in self.distance_sensors
        if names is None:
            names = list(self.contact_sensors.keys())

        # Retrieve data
        for contact_sensor_name in names:
            if contact_sensor_name not in self.contact_sensors:
                raise Exception(f"Sensor {contact_sensor_name} does not exists.")
            data = self.sensors_data[contact_sensor_name]
            seq_stamps.append((data.msg_frame_id, data.msg_stamp))
            contact_states[contact_sensor_name] = [data.contact_right, data.contact_left]
            # contact_states[contact_sensor_name] = data.contact_left

        return contact_states, seq_stamps
