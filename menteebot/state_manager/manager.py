import logging
from typing import Dict

from . import state_handlers
from .state_handlers import BaseManager

logger = logging.getLogger("menteebot")


class StateManager:
    def __init__(self, cfg, ros_node):
        self.cfg = cfg

        self.states: Dict[str, BaseManager] = dict()

        for name, state_cfg in self.cfg.items():
            logger.info(f"Initializing state - {name}")
            state_cls = getattr(state_handlers, state_cfg.cls)
            state_handler = state_cls(state_cfg, ros_node)
            self.states[name] = state_handler

    def update(self):
        for state in self.states.values():
            if hasattr(state, "update"):
                try:
                    state.update()
                except Exception as e:
                    logger.error(f"Unable to update state {state}.")
                    logger.error(e)
