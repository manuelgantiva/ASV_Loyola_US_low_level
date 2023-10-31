#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import random

from asv_interfaces.msg import StateObserver

class TestObserverNode(Node):
    def __init__(self):
        super().__init__("test_observer")
        self.declare_parameter("my_id", 0)

        self.my_id=  self.get_parameter("my_id").value

        self.timer_=self.create_timer(0.1, self.publish_asv_data)
        self.publisher_=self.create_publisher(StateObserver, "/control/state_observer", qos_profile_sensor_data)
        self.get_logger().info("Test Observer Node has been started")

    def publish_asv_data(self):
        msg = StateObserver()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.my_id)
        msg.point.x = round(random.uniform(-100,100),4)
        msg.point.y = round(random.uniform(-100,100),4)
        msg.point.z = round(random.uniform(-100,100),4)
        msg.velocity.x = round(random.uniform(-100,100),4)
        msg.velocity.y = round(random.uniform(-100,100),4)
        msg.velocity.z = round(random.uniform(-100,100),4)
        msg.disturbances.x = round(random.uniform(-100,100),4)
        msg.disturbances.y = round(random.uniform(-100,100),4)
        msg.disturbances.z = round(random.uniform(-100,100),4)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestObserverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
