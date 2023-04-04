import logging
import traceback

from common.srv import BrainStatus, TriggerListService
from rclpy.node import Node as RosNode
from std_srvs.srv import SetBool

# from ..neural_networks.networks.datatypes import NPDictType
# from ..neural_networks.networks_manager import NetworksManager
# from ..state_manager.manager import StateManager
from mentee_balance.state_manager.manager import StateManager
logger = logging.getLogger("menteebot")


class Brain:
    def __init__(self, cfg):

        self.cfg = cfg
        self.ros_node = RosNode("menteebot")
        self.state_manager = StateManager(cfg.states, self.ros_node)
        self.network_manager = NetworksManager(
            cfg.networks, self.state_manager, self.ros_node, self.cfg.frequency
        )

        # Stop/Start network loop
        self.do_net_loop = False
        # Should do "go_to_init" when no network loop
        self.go_to_init = False
        # Set current network to "main"
        self.current_net = "main"

        # Network loop service - inorder to start/stop the main loop
        self.net_loop_srv = self.ros_node.create_service(SetBool, "net_loop", self.set_net_loop)
        # Change network service
        self.change_net_srv = self.ros_node.create_service(
            TriggerListService, "change_net", self.change_net
        )
        # Get brain status
        self.get_status_srv = self.ros_node.create_service(
            BrainStatus, "brain_status", self.get_status
        )

        # Initiate the main network loop at 1/freq
        self.motion_timer = self.ros_node.create_timer(
            1 / self.cfg.frequency, self.perform_main_loop
        )

        logger.info("Network loop is turned OFF")

    def get_status(self, req: BrainStatus.Request, res: BrainStatus.Response):
        res.network = self.current_net
        res.go_to_init = self.go_to_init

        return res

    def change_net(self, req: TriggerListService.Request, res: TriggerListService.Response):
        if len(req.name) > 0:
            # Only use the first network for now
            network_name = req.name[0]
            if network_name in self.network_manager.networks.keys():
                self.current_net = network_name
                res.success = [True]
                res.message = [f"Changed to network {network_name}"]
            else:
                res.success = [False]
                res.message = [f"{network_name} is not in network_manager.networks"]
        else:
            res.success = [False]
            res.message = ["No network was given"]

        return res

    def set_net_loop(self, do_net_loop: SetBool.Request, response: SetBool.Response):
        self.do_net_loop = do_net_loop.data
        if self.do_net_loop:
            reset_response = self.network_manager.network_reset([self.current_net])
            cur_policy_name = list(reset_response.keys())[0]
            logger.info(
                f"Network loop is turned ON. Changing to '{cur_policy_name}' network. Performing reset."
            )
            logger.info(f"Changing to '{cur_policy_name}' network.")
            logger.info("Performing network reset.")

            response.success = reset_response[cur_policy_name][0]
            response.message = reset_response[cur_policy_name][1]
        else:
            logger.info("Network loop is turned OFF")
            response.success = True
            self.go_to_init = True
            response.message = "Network loop stopped"

        return response

    def perform_main_loop(self, event=None) -> None:
        if not self.do_net_loop:
            if self.go_to_init:
                self.state_manager.states["dofs"].stop()
                self.go_to_init = False
            return

        try:
            net_response: NPDictType = self.network_manager.network_query([self.current_net])
        except Exception as e:
            logger.error("Unable to query network with stack trace")
            logger.error(traceback.print_exc())
            return

        success, net_outputs, request_id = net_response[self.current_net]

        self.state_manager.states["dofs"].publish_state(action_dict=net_outputs, req_id=request_id)
