from collections import defaultdict
from functools import partial
from typing import Callable, Dict, List, Optional, Tuple, Type

from ...sensors import imus
from ...sensors.imus import BaseIMU, IMUData
from .base import BaseManager
from .datatypes import RigidBodyStateData


class RigidBodyStateManager(BaseManager):
    def __init__(self, cfg, ros_node):
        super(RigidBodyStateManager, self).__init__(cfg, ros_node)

        # Dictionary of rigid bodies
        self.rigid_bodies = defaultdict(RigidBodyStateData)

        # Dictionary of all IMUs
        self.imus = dict()

        # Init IMUs
        for name, imu_cfg in self.cfg.imus.items():
            # Hook the imu class and patch it with new callback function
            imu_cls: Type[BaseIMU] = self.hook_callback(getattr(imus, imu_cfg.cls))
            # Init imu class
            imu_handler: BaseIMU = imu_cls(name, imu_cfg, ros_node)
            # Save imu class in dictionary
            self.imus[name] = imu_handler

    def hook_callback(self, imu_cls: Type[BaseIMU]) -> Type[BaseIMU]:
        # Monkey patch the callback function
        # When triggered, it will also execute '_update_rigid_body_data'cdcd
        class Wrapper(imu_cls):
            def imu_callback(wrap_self, *args, **kwargs):
                # Partially initializing the function allows to put variables without execution
                callback_func = partial(super(Wrapper, wrap_self).imu_callback, *args, **kwargs)
                self._update_rigid_body_data(callback_func)

        return Wrapper

    def _update_rigid_body_data(self, callback_func: Callable[[], Tuple[str, IMUData]]):
        # We assume the callback function returns data
        name, imu_data = callback_func()

        # Update rigid bodies dictionary
        self.rigid_bodies[name].pos = imu_data.position
        self.rigid_bodies[name].rot = imu_data.quaternion
        self.rigid_bodies[name].lin_vel.local = imu_data.linear_velocity.local
        self.rigid_bodies[name].lin_vel.lab = imu_data.linear_velocity.lab
        self.rigid_bodies[name].ang_vel.local = imu_data.angular_velocity.local
        self.rigid_bodies[name].ang_vel.lab = imu_data.angular_velocity.lab
        self.rigid_bodies[name].acc.local = imu_data.acceleration.local
        self.rigid_bodies[name].acc.lab = imu_data.acceleration.lab
        self.rigid_bodies[name].msg_frame_id = imu_data.msg_frame_id
        self.rigid_bodies[name].msg_stamp = imu_data.msg_stamp

    def get_states(
        self,
        names: Optional[List[str]] = None,
        states: Optional[List[str]] = None,
        local_coor: bool = False,
    ) -> Tuple[Dict[str, List[float]], List]:
        # Init dict
        rigid_body_state = defaultdict(list)
        seq_stamps = []

        # If no names are provided, get all the named in self.rigid_bodies == All registered rigid bodies
        if names is None:
            names = list(self.rigid_bodies.keys())

        # If no state is provided, get all the DataClass Fields, which are the available states
        if states is None:
            # states = list(RigidBodyStateData.__dataclass_fields__.keys())
            states = ["pos", "pos", "lin_vel", "ang_vel", "acc"]

        # Retrieve data
        for rigid_body_name in names:
            if rigid_body_name not in self.rigid_bodies:
                raise Exception(f"Rigid-body {rigid_body_name} does not exists.")
            rigid_body_data = self.rigid_bodies[rigid_body_name]
            seq_stamps.append((rigid_body_data.msg_frame_id, rigid_body_data.msg_stamp))

            for state in states:
                state_value = getattr(rigid_body_data, state)
                if ("vel" in state) or ("acc" in state):
                    if local_coor:
                        state_value = state_value.local
                    else:
                        state_value = state_value.lab

                rigid_body_state[rigid_body_name] += state_value

        return rigid_body_state, seq_stamps


class RigidBodyStateManagerNoROS(BaseManager):
    def __init__(self, cfg, ros_node):
        super(RigidBodyStateManagerNoROS, self).__init__(cfg, ros_node)

        # Dictionary of rigid bodies
        self.rigid_bodies = defaultdict(RigidBodyStateData)

        # Dictionary of all IMUs
        self.imus = dict()

        # Init IMUs
        for name, imu_cfg in self.cfg.imus.items():
            # Hook the imu class and patch it with new callback function
            imu_cls: Type[BaseIMU] = getattr(imus, imu_cfg.cls)
            # Init imu class
            imu_handler: BaseIMU = imu_cls(name, imu_cfg, ros_node)
            # Save imu class in dictionary
            self.imus[name] = imu_handler

    def _update_rigid_body_data(self, imu_handler_name):
        # We assume the callback function returns data
        name, imu_data = self.imus[imu_handler_name].imu_callback(None)

        # Update rigid bodies dictionary
        self.rigid_bodies[name].pos = imu_data.position
        self.rigid_bodies[name].rot = imu_data.quaternion
        self.rigid_bodies[name].lin_vel.local = imu_data.linear_velocity.local
        self.rigid_bodies[name].lin_vel.lab = imu_data.linear_velocity.lab
        self.rigid_bodies[name].ang_vel.local = imu_data.angular_velocity.local
        self.rigid_bodies[name].ang_vel.lab = imu_data.angular_velocity.lab
        self.rigid_bodies[name].acc.local = imu_data.acceleration.local
        self.rigid_bodies[name].acc.lab = imu_data.acceleration.lab
        self.rigid_bodies[name].msg_frame_id = imu_data.msg_frame_id
        self.rigid_bodies[name].msg_stamp = imu_data.msg_stamp

    def update(self):
        for imu_handler_name in self.imus:
            self._update_rigid_body_data(imu_handler_name)

    def get_states(
        self,
        names: Optional[List[str]] = None,
        states: Optional[List[str]] = None,
        local_coor: bool = False,
    ) -> Tuple[Dict[str, List[float]], List]:
        # Init dict
        rigid_body_state = defaultdict(list)
        seq_stamps = []

        # If no names are provided, get all the named in self.rigid_bodies == All registered rigid bodies
        if names is None:
            names = list(self.rigid_bodies.keys())

        # If no state is provided, get all the DataClass Fields, which are the available states
        if states is None:
            # states = list(RigidBodyStateData.__dataclass_fields__.keys())
            states = ["pos", "pos", "lin_vel", "ang_vel", "acc"]

        # Retrieve data
        for rigid_body_name in names:
            if rigid_body_name not in self.rigid_bodies:
                raise Exception(f"Rigid-body {rigid_body_name} does not exists.")
            rigid_body_data = self.rigid_bodies[rigid_body_name]
            seq_stamps.append((rigid_body_data.msg_frame_id, rigid_body_data.msg_stamp))

            for state in states:
                state_value = getattr(rigid_body_data, state)
                if ("vel" in state) or ("acc" in state):
                    if local_coor:
                        state_value = state_value.local
                    else:
                        state_value = state_value.lab

                rigid_body_state[rigid_body_name] += state_value

        return rigid_body_state, seq_stamps
