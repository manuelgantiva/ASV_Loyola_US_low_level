#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import random

from asv_interfaces.msg import AsvObservation

class TestObserverNode(Node):
    def __init__(self):
        super().__init__("test_observer")
        self.timer_=self.create_timer(0.2, self.publish_asv_data)
        self.publisher_=self.create_publisher(AsvObservation, "asv_hat", 10)
        self.get_logger().info("Test Observer Node has been started")

    def publish_asv_data(self):
        my_info = AsvObservation()
        my_info.px= round(random.uniform(-100,100),4)
        my_info.py= round(random.uniform(-100,100),4)
        my_info.pz= round(random.uniform(-100,100),4)
        my_info.vx= round(random.uniform(-100,100),4)
        my_info.vy= round(random.uniform(-100,100),4)
        my_info.vz= round(random.uniform(-100,100),4)
        my_info.sx= round(random.uniform(-100,100),4)
        my_info.sy= round(random.uniform(-100,100),4)
        my_info.sz= round(random.uniform(-100,100),4)
        self.publisher_.publish(my_info)

def main(args=None):
    rclpy.init(args=args)
    node = TestObserverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
