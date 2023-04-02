import logging
import time
from abc import abstractmethod
from collections import defaultdict
from functools import partial
from typing import Callable, Dict, List, Optional, Tuple, Type

from candle_ros2.msg import MotionCommand

from menteebot.sensors.motors.datatypes import MotorData

from ...sensors import motors
from ...sensors.motors import BaseMotor
from ...utils.recorder import Recorder
from .base import BaseManager
from .datatypes import DoFStateData
from menteebot.state_manager.state_handlers import wrist_mapping

logger = logging.getLogger("menteebot")


class BaseDofStateManger(BaseManager):
    def __init__(self, cfg, *args, **kwargs):
        super(BaseDofStateManger, self).__init__(cfg, *args, **kwargs)

        # Dictionary of DoFs
        self.dofs = defaultdict(DoFStateData)

        # Dictionary of all motors_handlers
        self.motors_handlers: Dict[str, BaseMotor] = dict()
        # Init motors_handlers
        for name, motor_cfg in self.cfg.motors.items():
            self.init_motor_handler(name, motor_cfg)

        # Get mapping
        # self.motor_ID_to_name = dict(self.cfg.mapping.items())
        # self.motor_Name_to_ID = dict(map(reversed, self.motor_id2rigid_body.items()))

        self.recorder = Recorder(
            "dofs.csv",
            self.ros_node,
            fields_names=[
                "frame_id",
                "start_time",
                "get_data",
                "watchdog_times",
                "append_times",
                "all_watchdog_loop_time",
                "publish_time",
                "total_time",
            ],
        )

    @abstractmethod
    def init_motor_handler(self, name, motor_cfg):
        """Init motor handlers"""

    @abstractmethod
    def _update_dof_data(self, motor_section_name: str, **kwargs):
        """Update all dofs data from the motors"""

    def stop(self):
        for handler in self.motors_handlers.values():
            handler.go_to_init()

    def get_states(
            self, names: Optional[List[str]] = None, states: Optional[List[str]] = None, **kwargs
    ) -> Tuple[Dict[str, List[float]], List]:
        # Init dict
        dof_state = defaultdict(list)
        seq_stamps = []

        # If no names are provided, get all the named in self.dofs == All registered dofs
        if names is None:
            names = list(self.dofs.keys())

        # If no state is provided, get all the DataClass Fields, which are the available states
        if states is None:
            # states = list(DoFStateData.__dataclass_fields__.keys())
            states = ["pos", "vel", "torque"]

        # Retrieve data
        for dof_name in names:
            if dof_name not in self.dofs:
                raise Exception(f"DoF {dof_name} does not exists.")
            dof_data = self.dofs[dof_name]
            seq_stamps.append((dof_data.msg_seq, dof_data.msg_stamp))

            for state in states:
                dof_state[dof_name].append(getattr(dof_data, state))

        return dof_state, seq_stamps

    @abstractmethod
    def _get_action_class(self):
        """Return the class of actions that we send"""

    @abstractmethod
    def _update_dof_actions(
            self, action_dict, motor_section_name, motor_id, position, velocity, torque, kp, kd
    ):
        """Update actions sent to motor according to what is needed in this handler"""

    def publish_state(self, action_dict: Dict[str, float], req_id: int):
        start_time = time.time()
        self.recorder.add_to_buffer("start_time", time.time())

        # Init command
        motors_to_publish = defaultdict(self._get_action_class())

        # Aggregate data
        for dof_name in self.dofs.keys():
            with self.recorder.timeit("get_data", True):
                # Get motor name
                motor_section_name = self.dofs[dof_name].handler_name
                if motor_section_name == "":
                    logger.info(
                        f"Trying to publish to motor handler {motor_section_name}, "
                        "but it does not exists."
                    )
                    continue
                if dof_name not in action_dict:
                    logger.info(f"Trying to publish to dof {dof_name}, but no action was given")
                    continue

                # Get can id
                motor_handler = self.motors_handlers[motor_section_name]
                motor_id = motor_handler.rigid_body2motor_id[dof_name]
                wanted_pos = action_dict[dof_name]

                # Get torque and kp, kd if available
                wanted_kp = action_dict.get("kp_" + dof_name, None)
                wanted_kd = action_dict.get("kd_" + dof_name, None)
                wanted_torque = action_dict.get("torque_" + dof_name, 0.0)

            self._update_dof_actions(
                motors_to_publish,
                motor_section_name,
                motor_id,
                wanted_pos,
                0.0,  # velocity currently always 0.0
                wanted_torque,
                wanted_kp,
                wanted_kd,
            )

        with self.recorder.timeit("publish_time"):
            for motor_section_name, cmd in motors_to_publish.items():  # type: str, MotionCommand
                # roll = 0
                # pitch = 0

                if self.motors_handlers[motor_section_name].enable:
                    #TODO: expand this function to fit both arms not only 204-205
                    # rool_id_right =  211
                    # pitch_id_right = 212
                    yaw_id_left = 204
                    pitch_id_left = 205

                    # overriding cmd value from actual input - roll and pitch angles , to actual motor values after
                    # converting it on wrist_mapping.get_motors function with Adi's formula.

                    yaw_left = cmd[yaw_id_left]["position"]
                    pitch_left = cmd[pitch_id_left]["position"]
                    #
                    #
                    cmd[yaw_id_left]['position'], cmd[pitch_id_left]['position'] = wrist_mapping.get_motors_from_pitch_yaw(pitch_left, yaw_left)

                    self.motors_handlers[motor_section_name].apply(req_id, cmd)
        self.recorder.add_to_buffer("total_time", time.time() - start_time)
        self.recorder.flush()


class DofStateManagerROS(BaseDofStateManger):
    def __init__(self, cfg, *args, **kwargs):
        super(DofStateManagerROS, self).__init__(cfg, *args, **kwargs)

    def init_motor_handler(self, name, motor_cfg):
        # Hook the motor class and patch it with new callback function
        motor_cls: Type[BaseMotor] = self.hook_callback(name, getattr(motors, motor_cfg.cls))
        # Init motor class
        motor_handler: BaseMotor = motor_cls(name, motor_cfg, self.ros_node)
        # Save motor class in dictionary
        self.motors_handlers[name] = motor_handler

    def hook_callback(self, motor_section: str, motor_cls: Type[BaseMotor]) -> Type[BaseMotor]:
        # Monkey patch the callback function
        # When triggered, it will also execute '_update_dof_data'
        class Wrapper(motor_cls):
            def get_data(wrap_self, *args, **kwargs):
                # Partially initializing the function allows to put variables without execution
                callback_func = partial(super(Wrapper, wrap_self).get_data, *args, **kwargs)
                self._update_dof_data(motor_section, callback_func)

        return Wrapper

    def _update_dof_data(
            self, motor_section_name: str, callback_func: Callable[[], Tuple[str, MotorData]]
    ):
        # We assume the callback function yields data from a 'for' loop
        for motor_id, motor_data in callback_func():
            dof_name = self.motors_handlers[motor_section_name].motor_id2rigid_body[float(motor_id)]
            self.dofs[dof_name].pos = motor_data.position
            # print("@@@@  this is {} motor pos  {}".format(dof_name, self.dofs[dof_name].pos))
            # print(motor_data.position)
            self.dofs[dof_name].vel = motor_data.velocity
            # self.dofs[dof_name].savgol_vel = motor_data.savgol_vel
            # self.dofs[dof_name].kalman_vel = motor_data.kalman_vel
            self.dofs[dof_name].torque = motor_data.torque
            self.dofs[dof_name].msg_seq = motor_data.msg_frame_id
            self.dofs[dof_name].msg_stamp = motor_data.msg_stamp
            if not self.dofs[dof_name].handler_name:
                self.dofs[dof_name].handler_name = motor_section_name

    def _get_action_class(self):
        return MotionCommand

    def _update_dof_actions(
            self, action_dict, motor_section_name, motor_id, position, velocity, torque, kp, kd
    ):
        with self.recorder.timeit("append_times", True):
            action_dict[motor_section_name].drive_ids.append(motor_id)
            action_dict[motor_section_name].target_position.append(float(position))
            action_dict[motor_section_name].target_velocity.append(velocity)
            action_dict[motor_section_name].target_torque.append(torque)
            kp = (
                kp
                if kp is not None
                else self.motors_handlers[motor_section_name].default_kp[motor_id]
            )
            kd = (
                kd
                if kd is not None
                else self.motors_handlers[motor_section_name].default_kd[motor_id]
            )
            action_dict[motor_section_name].kp.append(kp)
            action_dict[motor_section_name].kd.append(kd)


class DofStateManagerNoROS(BaseDofStateManger):
    def init_motor_handler(self, name, motor_cfg):
        motor_cls: Type[BaseMotor] = getattr(motors, motor_cfg.cls)
        # Init motor class
        motor_handler: BaseMotor = motor_cls(name, motor_cfg)
        # Save motor class in dictionary
        self.motors_handlers[name] = motor_handler
        # Update states
        self._update_dof_data(name)

    def _update_dof_data(self, motor_section_name: str):

        for dof_name, motor_data in self.motors_handlers[motor_section_name].get_data():
            # Set DoF data
            self.dofs[dof_name].msg_seq = motor_data["seq"]
            self.dofs[dof_name].msg_stamp = motor_data["time"]

            self.dofs[dof_name].pos = motor_data["position"]

            self.dofs[dof_name].vel = motor_data["velocity"]
            # self.dofs[dof_name].savgol_vel = motor_data["savgol_vel"]
            # self.dofs[dof_name].kalman_vel = motor_data["kalman_vel"]
            self.dofs[dof_name].torque = motor_data["torque"]

            # if dof_name=="right_shoulder_roll":
            #     print("{0} RT pos is {1} , torque is {2}".format(dof_name, motor_data["position"],motor_data["torque"]))
            #     # with open("/tmp/log-test.csv", "a") as f:
            #     #     f.write(f"{dof_name},{motor_data['position']},{motor_data['torque']}\n")
            #
            #     # print("{0} RT torque is {1}".format(dof_name, motor_data["torque"]))
            #     # print("###########################################################")

            if not self.dofs[dof_name].handler_name:
                self.dofs[dof_name].handler_name = motor_section_name

    def update(self):
        for motor_handler_name in self.motors_handlers:
            self._update_dof_data(motor_handler_name)

    def _get_action_class(self):
        return dict

    def _update_dof_actions(
            self, action_dict, motor_section_name, motor_id, position, velocity, torque, kp, kd
    ):
        with self.recorder.timeit("append_times", True):
            command_to_send = dict()
            command_to_send["position"] = position
            command_to_send["velocity"] = velocity
            command_to_send["torque"] = torque
            command_to_send["kp"] = (
                kp
                if kp is not None
                else self.motors_handlers[motor_section_name].default_kp[motor_id]
            )
            command_to_send["kd"] = (
                kd
                if kd is not None
                else self.motors_handlers[motor_section_name].default_kd[motor_id]
            )

            action_dict[motor_section_name][motor_id] = command_to_send
