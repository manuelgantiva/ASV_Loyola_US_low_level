#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from digi.xbee.devices import XBeeDevice
import struct

from geometry_msgs.msg import Vector3

class XbeeSlaveNode(Node):
    def __init__(self):
        super().__init__("xbee_slave")
        self.declare_parameter("my_id", "ASV0")
        my_id = self.get_parameter("my_id").get_parameter_value().string_value
        try:
            self.xbee = XBeeDevice("/dev/xbee_usb", 115200)
            self.get_logger().info("\033[32mSerial Xbee port opened successfully...\033[0m")
        except Exception as e:
            self.get_logger().info("\033[31mSerial Xbbe port opening failure\033[0m")
        self.xbee.open()
        self.xbee.add_data_received_callback(self.callback_received_data)  # Agregar el callback de recepción de datos

        self.publisher_ = self.create_publisher(Vector3, "/" + my_id +"/control/ref_master", 10)
        self.subscriber_ = self.create_subscription(Vector3, "/" + my_id +"/control/error_mlc",
                        self.callback_mlc,1)
        self.get_logger().info("Xbee slave Node in " + my_id + " has been started")

    def callback_mlc(self, msg: Vector3):

        byte_array = bytearray()  # Create byte array
        byte_array = struct.pack('!e', msg.z) # w is packed into an array
        try:
            self.xbee.send_data_broadcast(byte_array)  # Send data
        except Exception as e:
            self.get_logger().info("Data was not sent")  # Catches exception and returns unsuccessful

    
    def callback_received_data(self, xbee_message):
        # self.get_logger().info("Data receive")  # Catches exception and returns unsuccessful
        byte_array = xbee_message.data  # Extraemos el dato del mensaje
        data_f = []  # Creamos la lista que contendrá los valores decodificados
        data_f = list(struct.unpack('!e', byte_array))

        info_rcv = Vector3()
        info_rcv.y = data_f[0]
        self.publisher_.publish(info_rcv)


def main(args=None):
    rclpy.init(args=args)
    node = XbeeSlaveNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
