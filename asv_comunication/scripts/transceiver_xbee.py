#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


from digi.xbee.devices import XBeeDevice
import struct
import time

from asv_interfaces.msg import StateObserver, XbeeObserver

class TransceiverXbeeNode(Node):
    def __init__(self):
        super().__init__("xbee_node")
        self.xbee = XBeeDevice("/dev/ttyUSB0", 115200)
        self.states = []
        self.count_ = 0
        self.xbee.open()
        self.xbee.add_data_received_callback(self.callback_received_data)  # Agregar el callback de recepción de datos
        self.publisher_ = self.create_publisher(XbeeObserver, "xbee_observer", 1)
        self.subscriber_ = self.create_subscription(StateObserver, "/control/state_observer",
                        self.callback_state_observer,qos_profile_sensor_data)
        self.timer_ = self.create_timer(0.1, self.publish_states)
        self.get_logger().info("Transceiver Xbee Node has been started")

    def publish_states(self):
        msg=XbeeObserver()
        msg.counter=self.count_
        msg.states=self.states
        self.publisher_.publish(msg)
        self.count_ = 0
        del self.states[:]


    def callback_state_observer(self, msg):
        timenow = float(time.time())
        id = 1

        byte_array = bytearray()  # Create byte array

        byte_array = struct.pack('!e e e e e e 1s',
                             msg.point.x, msg.point.y, msg.point.z,
                             msg.velocity.x, msg.velocity.y, msg.velocity.z,
                             msg.header.frame_id.encode('utf-8')) # The data is packed into an array

        try:
            self.xbee.send_data_broadcast(byte_array)  # Send data
        except Exception as e:
            self.get_logger().info("Data was not sent")  # Catches exception and returns unsuccessful

    
    def callback_received_data(self, xbee_message):
        byte_array = xbee_message.data  # Extraemos el dato del mensaje
        data_f = []  # Creamos la lista que contendrá los valores decodificados
        data_f = list(struct.unpack('!e e e e e e 1s', byte_array))

        info_rcv = StateObserver()
        info_rcv.point.x = data_f[0]
        info_rcv.point.y = data_f[1]
        info_rcv.point.z = data_f[2]
        info_rcv.velocity.x = data_f[3]
        info_rcv.velocity.y = data_f[4]
        info_rcv.velocity.z = data_f[5]
        info_rcv.disturbances.x = 0.0
        info_rcv.disturbances.y = 0.0
        info_rcv.disturbances.z = 0.0
        info_rcv.header.stamp.sec = 0
        info_rcv.header.stamp.nanosec = 0
        info_rcv.header.frame_id = data_f[6].decode('utf-8')
        self.states.append(info_rcv)
        self.count_=self.count_+1

def main(args=None):
    rclpy.init(args=args)
    node = TransceiverXbeeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
