import logging
from typing import Dict, List, Type

from common.srv import TriggerListService
from rclpy.node import Node as RosNode

from ..state_manager.manager import StateManager
from . import networks
from .networks.base import BaseNetwork
from .networks.datatypes import NPDictType

logger = logging.getLogger("menteebot")


class NetworksManager:
    def __init__(self, cfg, state_manager: StateManager, ros_node: RosNode, frequency: int):
        self.cfg = cfg

        # Initialising all networks and save then in a dictionary
        self.networks: Dict[str, BaseNetwork] = {}
        for net_name, net_cfg in self.cfg.items():
            network_cls: Type[BaseNetwork] = getattr(networks, net_cfg.cls)
            if net_name in self.networks:
                logger.info(f"Overriding network {net_name}!")
            self.networks[net_name] = network_cls(
                net_name, net_cfg, state_manager, ros_node, frequency
            )

        # We assume this class is used under the 'brain' node
        # So no need to init a new node
        self.net_reset_srv = ros_node.create_service(
            TriggerListService, "net_reset", self.network_reset_srv
        )

    def network_query(self, names: List[str]):
        response = dict()

        for name in names:
            success = False
            if name not in self.networks:
                logger.info(f"Received a network query request fo unknown network name: {name}")
                net_response: NPDictType = {}
                request_id = -1
            else:
                # Network inputs
                net_response, request_id = self.networks[name].apply()  # type NPDictType, int
                success = True
            response[name] = success, net_response, request_id

        return response

    def network_reset_srv(self, req, response):
        successes = []
        strings = []
        for name in req.name:
            if name not in self.networks:
                logger.info(f"Received a network reset request fo unknown network name: {name}")
                successes.append(False)
                strings.append("Reset error... no network found")
            else:
                logger.info(f"Reset network : {name}")
                success, msg = self.networks[name].reset()
                successes.append(success)
                strings.append(msg)
        response.success = success
        response.message = strings
        return response

    def network_reset(self, names: List[str]):
        response = dict()

        for name in names:
            success = False
            if name not in self.networks:
                logger.info(f"Received a network reset request fo unknown network name: {name}")
                message = "Reset error... no network found"
            else:
                logger.info(f"Reset network : {name}")
                success, message = self.networks[name].reset()
            response[name] = success, message

        return response
