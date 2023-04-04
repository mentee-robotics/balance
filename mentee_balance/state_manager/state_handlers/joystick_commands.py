from collections import defaultdict
from dataclasses import dataclass
from functools import partial
from typing import Callable, Dict, List, Optional, Tuple, Type


from .base import BaseManager
from .datatypes import CommandData
import  sys

sys.path.append("../../..")
from sensors import teleops
from sensors.teleops.base import BaseCommand, CommandTwistData
class JoystickCommandStateHandler(BaseManager):
    def __init__(self, cfg, ros_node):
        super(JoystickCommandStateHandler, self).__init__(cfg, ros_node)

        # Dictionary of rigid bodies
        self.command = CommandData()

        # Hook the imu class and patch it with new callback function
        commander_cls: Type[BaseCommand] = self.hook_callback(
            getattr(teleops, cfg.command.joystick_commands.cls)
        )
        # Init imu class
        self.commander: BaseCommand = commander_cls(
            cfg.command.joystick_commands.name, cfg.command.joystick_commands, ros_node
        )

    def hook_callback(self, command_cls: Type[BaseCommand]) -> Type[BaseCommand]:
        # Monkey patch the callback function
        # When triggered, it will also execute '_update_rigid_body_data'
        class Wrapper(command_cls):
            def command_callback(wrap_self, *args, **kwargs):
                # Partially initializing the function allows to put variables without execution
                callback_func = partial(super(Wrapper, wrap_self).command_callback, *args, **kwargs)
                self._update_command(callback_func)

        return Wrapper

    def _update_command(self, callback_func: Callable[[], CommandData]):
        # We assume the callback function returns data
        command = callback_func()
        self.command.msg_frame_id = int(command.msg_frame_id)
        self.command.msg_stamp = command.msg_stamp
        self.command.data = [
            command.lin_vel_x,
            command.lin_vel_y,
            command.heading,
            command.foot_height,
        ]

    def get_states(
        self,
        names: Optional[List[str]] = None,
        states: Optional[List[str]] = None,
        start: Optional[int] = None,
        end: int = 1,
        **kwargs,
    ) -> Tuple[List[float], List]:
        # Init dict

        # Retrieve data
        seq_stamps = [self.command.msg_frame_id, self.command.msg_stamp]

        command_state = self.command.data[slice(start, end)]

        return command_state, seq_stamps
