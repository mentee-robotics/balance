import time

import rclpy
from geometry_msgs.msg import Quaternion, QuaternionStamped, Vector3, Vector3Stamped
from rclpy.node import Node as RosNode
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu


class ImuPublisher:
    def __init__(self):
        self.ros_node = RosNode("imu_sim")
        self.publisher_imu = self.ros_node.create_publisher(Imu, "/imu/data", QoSProfile(depth=10))
        self.publisher_free_acc = self.ros_node.create_publisher(
            Vector3Stamped, "/filter/free_acceleration", QoSProfile(depth=10)
        )
        self.frame_id = 0
        self.motion_timer = self.ros_node.create_timer(1 / 200, self.sim_loop)

    def sim_loop(self):
        new_imu = Imu()
        new_imu.header.frame_id = str(self.frame_id)
        new_imu.header.stamp = self.ros_node.get_clock().now().to_msg()
        new_imu.orientation.x = 0.0
        new_imu.orientation.y = 0.0
        new_imu.orientation.z = 0.0
        new_imu.orientation.w = 1.0
        new_imu.angular_velocity.x = 0.0
        new_imu.angular_velocity.y = 0.0
        new_imu.angular_velocity.z = 0.0
        new_imu.linear_acceleration.x = 0.0
        new_imu.linear_acceleration.y = 0.0
        new_imu.linear_acceleration.z = 0.0
        self.publisher_imu.publish(new_imu)
        # free_acc = Vector3Stamped()
        # free_acc.header.frame_id = str(self.frame_id)
        # free_acc.header.stamp = self.ros_node.get_clock().now().to_msg()
        # free_acc.vector.x = 0.0
        # free_acc.vector.y = 0.0
        # free_acc.vector.z = 0.0
        # self.publisher_free_acc.publish(free_acc)
        self.frame_id += 1


def main():
    rclpy.init()
    imu_node = ImuPublisher()
    rclpy.spin(imu_node.ros_node)


if __name__ == "__main__":
    main()
