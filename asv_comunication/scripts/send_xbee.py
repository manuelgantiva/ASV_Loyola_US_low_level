#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from digi.xbee.devices import XBeeDevice 
import time
import struct
import random


class SendXbeeNode(Node):
    def __init__(self):
        super().__init__("send_xbee")
        self.xbee= XBeeDevice("/dev/ttyUSB0",9600)
        self.xbee.open()
        self.timer_=self.create_timer(0.2, self.send_data)
        self.get_logger().info("Send Xbee Node has been started")

    def send_data(self):
        pos_x= round(random.uniform(-100,100),4)
        pos_y= round(random.uniform(-100,100),4)
        pos_z= round(random.uniform(-100,100),4)
        PWM_l= round(random.uniform(-100,100),4)
        PWM_r= round(random.uniform(-100,100),4)
        data_1= round(random.uniform(-100,100),4)
        data_2= round(random.uniform(-100,100),4)
        tiempo= float(time.time())

        data = [pos_x,pos_y,pos_z,PWM_l,PWM_r,data_1,data_2,tiempo] #Los datos que queremos enviar

        byte_array = bytearray() #Creamos un array de bytes que contendrá todos los números en punto flotante

        for numeros in data:
            if numeros != tiempo:
                bytes_data = self.f2bytes(numeros) #Convertimos en bytes todos los números de 'data'
                byte_array.extend(bytes_data) #Vamos añadiendo cada cadena de bytes al 'byte array'
            else:
                tiempo_data = self.d2bytes(numeros)
                byte_array.extend(tiempo_data)

        self.xbee.send_data_broadcast(byte_array) #Enviamos el dato a todos los nodos de la red 

    def f2bytes(self, f):
        bytes_data = struct.pack('!f', f) #Empaquetamos los datos como bytes
        return bytes_data

    def d2bytes(self, d):
        tiempo_data = struct.pack('d', d) #Empaquetamos los datos como bytes
        return tiempo_data

def main(args=None):
    rclpy.init(args=args)
    node = SendXbeeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
