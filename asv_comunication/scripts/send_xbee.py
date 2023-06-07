#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from digi.xbee.devices import XBeeDevice 
import struct
import time

from asv_interfaces.msg import AsvObservation
from asv_interfaces.srv import SendAsvObservation

class SendXbeeNode(Node):
    def __init__(self):
        super().__init__("send_xbee")
        #self.xbee= XBeeDevice("/dev/ttyUSB0",115200)
        #self.xbee.open()
        self.subscriber_ = self.create_subscription(AsvObservation, "asv_hat",self.callback_asv_hat,10)
        self.get_logger().info("Send Xbee Node has been started")
    
    def callback_asv_hat(self, msg):
        timenow= float(time.time())
        data = [msg.px, msg.py, msg.pz, msg.vx, msg.vy, msg.vz, msg.sx, msg.sy, msg.sz, timenow] # The data is packed into an array
        
        byte_array = bytearray() # Create byte array

        for numero in data:
            if numero != timenow:
                bytes_data = self.f2bytes(numero) # Convert to a byte array
                byte_array.extend(bytes_data) # Add to array
            else:
                tiempo_data = self.d2bytes(numero)
                byte_array.extend(tiempo_data)

        try:
            self.xbee.send_data_broadcast(byte_array) # Send data
            self.get_logger().info("Dates was send")
        except Exception as e: 
            self.get_logger().info("Dates was not send") # Catches exception and returns unsuccessful

    def f2bytes(self, f):
        bytes_data = struct.pack('!f', f) # Pack the data as bytes
        return bytes_data

    def d2bytes(self, d):
        tiempo_data = struct.pack('d', d) # Pack the data as bytes
        return tiempo_data

def main(args=None):
    rclpy.init(args=args)
    node = SendXbeeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()