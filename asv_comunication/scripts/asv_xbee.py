#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

import time

from asv_interfaces.msg import AsvObservation
from asv_interfaces.srv import SendAsvObservation


class AsvXbeeNode(Node): 
    def __init__(self):
        super().__init__("asv_xbee")
        self.subscriber_ = self.create_subscription(
                            AsvObservation, "asv_hat",self.callback_asv_hat,10)
        self.get_logger().info("Asv Xbee Node has been started")

    def callback_asv_hat(self, msg):
        timenow= float(time.time())
        self.call_send_asv_observation(msg, 4, timenow)

    def call_send_asv_observation(self, asv_info, id, time):
        client = self.create_client(SendAsvObservation, "send_asv_observation")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server Send Xbee... ")
        
        request = SendAsvObservation.Request()
        
        request.data.states = asv_info
        request.data.time = time
        request.data.id = id

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_send_asv_observation_resp))
    
    def callback_send_asv_observation_resp(self, future):
        try:
            response = future.result()
            self.get_logger().info("Response = " + str(response.success))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = AsvXbeeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()