#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import time

from asv_interfaces.msg import StateObserver, XbeeObserver, StateNeighbor

class TestXbeeNode(Node):
    def __init__(self):
        super().__init__("test_xbee_node")
        self.states = []
        self.count_ = 1

        info_rcv = StateNeighbor()
        info_rcv.point.x = -5.0
        info_rcv.point.y = 3.0
        info_rcv.point.z = 0.0
        info_rcv.velocity.x = 0.5
        info_rcv.velocity.y = 1.2
        info_rcv.velocity.z = 0.3
        info_rcv.id = "5"
        self.states.append(info_rcv)

        self.publisher_ = self.create_publisher(XbeeObserver, "/comunication/xbee_observer", 1)
        self.timer_ = self.create_timer(0.1, self.publish_states)
        self.get_logger().info("Test Transceiver Xbee Node has been started")

    def publish_states(self):
        msg=XbeeObserver()
        msg.counter=self.count_
        msg.states=self.states
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestXbeeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
