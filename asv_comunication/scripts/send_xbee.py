#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from digi.xbee.devices import XBeeDevice 
import struct

from asv_interfaces.msg import AsvObservation
from asv_interfaces.srv import SendAsvObservation

class SendXbeeNode(Node):
    def __init__(self):
        super().__init__("send_xbee")
        self.xbee= XBeeDevice("/dev/ttyUSB0",115200)
        self.xbee.open()
        self.server_=self.create_service(SendAsvObservation, "send_asv_observation", self.handle_send_asv_observation)
        self.get_logger().info("Send Xbee Node has been started")
    
    def handle_send_asv_observation(self, request, response):
        data = [request.data.states.px, request.data.states.py, request.data.states.pz, request.data.states.vx, request.data.states.vy, 
                request.data.states.vz, request.data.states.sx, request.data.states.sy, request.data.states.sz, request.data.time] # The data is packed into an array
        
        byte_array = bytearray() # Create byte array

        for numero in data:
            if numero != request.data.time:
                bytes_data = self.f2bytes(numero) # Convert to a byte array
                byte_array.extend(bytes_data) # Add to array
            else:
                tiempo_data = self.d2bytes(numero)
                byte_array.extend(tiempo_data)

        try:
            self.xbee.send_data_broadcast(byte_array) # Send data
            response.success = True
            self.get_logger().info("Dates was send")
        except Exception as e: 
            self.get_logger().info("Dates was not send") # Catches exception and returns unsuccessful
            response.success = False
        return response

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