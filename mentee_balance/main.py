#!/usr/bin/env python3
import argparse
import logging

import rclpy
from hydra import compose, initialize
from omegaconf import OmegaConf

from brain.brain import Brain

from utils.logger import configure_logging
# PYTHONUNBUFFERED=1;LD_LIBRARY_PATH=/home/administrator/code/ros2_workspace/install/state_estimation_ros2/lib:/home/administrator/code/ros2_workspace/install/cpp_ros_node_template/lib:/home/administrator/code/ros2_workspace/install/common/lib:/home/administrator/code/ros2_workspace/install/candle_ros2/lib:/opt/ros/foxy/opt/yaml_cpp_vendor/lib:/opt/ros/foxy/opt/rviz_ogre_vendor/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/opt/ros/foxy/lib

# Press the green button in the gutter to run the script.
def main():
    configure_logging()

    logger = logging.getLogger("menteebot")

    logger.info(":robot: Starting 'Brain :brain:' node :robot:")

    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", default="../configs/balance.yaml", help="Configuration file")

    args, unknown = parser.parse_known_args()

    # Configuring hydra
    yaml_path_split = args.yaml.split("/")
    config_path = "/".join(yaml_path_split[:-1])
    config_name = yaml_path_split[-1][:-5]
    with initialize(config_path=config_path, job_name="menteebot_app"):
        yaml_conf = compose(config_name=config_name, overrides=["hydra.run.dir=/tmp"])
        # Struct to normal :)
        yaml_conf = OmegaConf.to_container(yaml_conf)
        yaml_conf = OmegaConf.create(yaml_conf)
        # OmegaConf.save(yaml_conf, "../../menteebot/configs/master_config.yaml")

    cfg = yaml_conf

    logger.info(cfg)



    rclpy.init()
    brain = Brain(cfg)
    rclpy.spin(brain.ros_node)

if __name__ == "__main__":
    main()
