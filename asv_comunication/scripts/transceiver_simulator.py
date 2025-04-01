#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from copy import deepcopy
from collections import deque

from asv_interfaces.msg import StateObserver, StateNeighbor

class TransceiverSimulator(Node):
    def __init__(self):
        super().__init__("transceiver_node")
        self.declare_parameter("my_id", "ASV0")
        self.declare_parameter("worker_mode", 0)  # 0: Master, 1, 2, ..: Slave

        self.my_string_id = self.get_parameter("my_id").get_parameter_value().string_value
        self.worker_mode = self.get_parameter("worker_mode").get_parameter_value().integer_value

        try:
            self.xbee = self.create_publisher(StateNeighbor, "/simulated_env", qos_profile_sensor_data) # un topico en simulacion 
            self.get_logger().info("\033[32mSerial Sim port opened successfully...\033[0m")
        except Exception as e:
            self.get_logger().info("\033[31mSerial Sim port opening failure\033[0m")

        self.states = deque([])
        self.subscriber_xbee = self.create_subscription(StateNeighbor, "/simulated_env",
                        self.callback_received_data,qos_profile_sensor_data)
        if self.worker_mode == 0:
            self.publisher_hlc = self.create_publisher(StateNeighbor, "/" + self.my_string_id +"/neighbors/state_observer", 1)
            self.subscriber_state = self.create_subscription(StateNeighbor, "/" + self.my_string_id +"/neighbors/output_hlc",
                            self.callback_state_observer,qos_profile_sensor_data)
        else:
            self.publisher_hlc = self.create_publisher(StateObserver, "/" + self.my_string_id +"/control/output_hlc", qos_profile_sensor_data)
            self.subscriber_state = self.create_subscription(StateObserver, "/" + self.my_string_id +"/observer/state_observer",
                            self.callback_state_observer,qos_profile_sensor_data)
            self.timer_ = self.create_timer(0.1, self.publish_incoming_msgs)
        self.get_logger().info("Transceiver Simulator Node in " + self.my_string_id + " has been started in mode " + str(self.worker_mode))

    def callback_state_observer(self, msg):
        """
        # TODO: cambiar a Simular creación de mensaje
        byte_array = bytearray()  # Create byte array
        byte_array = struct.pack('!e e e e e e 1s',
                             msg.point.x, msg.point.y, msg.point.z,
                             msg.velocity.x, msg.velocity.y, msg.velocity.z,
                             msg.header.frame_id.encode('utf-8')) # probablemente self.my_string_id.encode('utf-8') es más rápido
        """
        # try:
        my_msg = StateNeighbor()
        if self.worker_mode == 0:
            my_msg.id = msg.id
        elif self.worker_mode == 1:
            my_msg.id = self.my_string_id
        my_msg.point = msg.point
        my_msg.velocity = msg.velocity
        my_msg.msg_from = self.worker_mode
        self.xbee.publish(my_msg)
        # elif self.worker_mode == 1:
        #     my_msg = StateNeighbor()
        #     my_msg.id = self.my_string_id
        #     my_msg.point = msg.point
        #     my_msg.velocity = msg.velocity
        #     my_msg.msg_from = self.worker_mode
        #     self.xbee.publish(my_msg)
        # except Exception as e:
        #     self.get_logger().info("Data was not sent")  # Catches exception and returns unsuccessful

 
    def callback_received_data(self, xbee_message):
        if xbee_message.msg_from != self.worker_mode:
            if self.worker_mode == 0 and  xbee_message.id != self.my_string_id:
                msg = deepcopy(xbee_message)
                self.states.append(msg)
                self.publish_incoming_msgs()
            elif xbee_message.id == str(self.worker_mode):
                """
                byte_array = xbee_message.data  # Extraemos el dato del mensaje
                data_f = []  # Creamos la lista que contendrá los valores decodificados
                data_f = list(struct.unpack('!e e e e e e 1s', byte_array))
                info_rcv = StateNeighbor()
                info_rcv.point.x = data_f[0]
                info_rcv.point.y = data_f[1]
                info_rcv.point.z = data_f[2]
                info_rcv.velocity.x = data_f[3]
                info_rcv.velocity.y = data_f[4]
                info_rcv.velocity.z = data_f[5]
                info_rcv.id = data_f[6].decode('utf-8')
                self.states.append(info_rcv)
                """
                msg = deepcopy(xbee_message)
                msg.id = self.my_string_id
                self.states.append(msg)


   
    def publish_incoming_msgs(self):
        if self.worker_mode == 0:
            while(len(self.states) > 0):
                msg = self.states.popleft()
                self.publisher_hlc.publish(msg)
        elif len(self.states) > 0:
            msg = self.states.popleft()
            my_msg = StateObserver()
            my_msg.header.frame_id = self.my_string_id

            my_msg.point = msg.point
            my_msg.velocity = msg.velocity
            self.publisher_hlc.publish(my_msg)
                

def main(args=None):
    rclpy.init(args=args)
    node = TransceiverSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
