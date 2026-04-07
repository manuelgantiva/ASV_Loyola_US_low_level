#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from copy import deepcopy
from collections import deque

from asv_interfaces.msg import StateObserver, StateNeighbor

class LeaderSimulator(Node):
    def __init__(self):
        super().__init__("transceiver_node")
        self.declare_parameter("my_id", "ASV0")
        self.declare_parameter("worker_mode", 1)  # 0: Master, 1, 2, ..: Slave

        self.my_string_id = self.get_parameter("my_id").get_parameter_value().string_value
        self.worker_mode = self.get_parameter("worker_mode").get_parameter_value().integer_value

        try:
            self.xbee = self.create_publisher(StateNeighbor, "/simulated_env", qos_profile_sensor_data) # un topico en simulacion 
            self.get_logger().info("\033[32mSerial Sim port opened successfully...\033[0m")
        except Exception as e:
            self.get_logger().info("\033[31mSerial Sim port opening failure\033[0m")

        self.states = deque([])
       
        self.subscriber_state = self.create_subscription(StateObserver, "/" + self.my_string_id + "/observer/state_observer", self.callback_state_observer, qos_profile_sensor_data)
        
        self.get_logger().info("Transceiver Simulator Node in " + self.my_string_id + " has been started in mode " + str(self.worker_mode))

    def callback_state_observer(self, msg):
        my_msg = StateNeighbor()
        my_msg.id = self.my_string_id
        my_msg.point = msg.point
        my_msg.velocity = msg.velocity
        my_msg.msg_from = self.worker_mode
        self.xbee.publish(my_msg)
        """
        # TODO: cambiar a Simular creación de mensaje
        byte_array = bytearray()  # Create byte array
        byte_array = struct.pack('!e e e e e e 1s',
                             msg.point.x, msg.point.y, msg.point.z,
                             msg.velocity.x, msg.velocity.y, msg.velocity.z,
                             msg.header.frame_id.encode('utf-8')) # probablemente self.my_string_id.encode('utf-8') es más rápido
        """
        # try:
        # elif self.worker_mode == 1:
        #     my_msg = StateNeighbor()
        #     my_msg.id = self.my_string_id
        #     my_msg.point = msg.point
        #     my_msg.velocity = msg.velocity
        #     my_msg.msg_from = self.worker_mode
        #     self.xbee.publish(my_msg)
        # except Exception as e:
        #     self.get_logger().info("Data was not sent")  # Catches exception and returns unsuccessful
                

def main(args=None):
    rclpy.init(args=args)
    node = LeaderSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()