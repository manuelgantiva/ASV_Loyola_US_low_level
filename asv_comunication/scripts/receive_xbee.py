#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from digi.xbee.devices import XBeeDevice 
import time
import struct

class ReceiveXbeeNode(Node):
    def __init__(self):
        super().__init__("receive_node")
        device2=XBeeDevice("/dev/ttyUSB0",9600)
        device2.open() #Abrimos la comunicación
        device2.add_data_received_callback(self.my_data_received_callback)
        self.get_logger().info("Receive Xbee Node has been started")

    def my_data_received_callback(self, xbee_message):
        byte_array=xbee_message.data #Extraemos el dato del mensaje
        data_f = [] #Creamos la lista que contendrá los valores en flotante decodificados
        data_f= self.bytes2f(byte_array, data_f) #Ejecutamos la función que decodifica los bytes recogidos y los vuelve a convertir en punto flotante
        data_f= self.bytes2d(byte_array, data_f)
        data_f.append(time.time()-data_f[7])
        data_f.append(time.time())
        print("\033[91mEl dato recibido es:\033[0m")
        print(data_f)


    def bytes2f(self, byte_array, data_f):
        for i in range(0, len(byte_array)-8, 4):
            numero_b = byte_array[i:i+4] #Vamos a ir desempaquetando cada número flotante por separado
            # Unpack the chunk as a single float value
            numero_f = struct.unpack('!f', numero_b)[0] #Unpack solo trabaja con 4 bytes a la vez, por eso la necesidad de dividir nuestro frame
            # Append the float value to the result list
            data_f.append(numero_f) #Vamos añadiendo cada numero decodificado a una lista
        return data_f

    def bytes2d(self, byte_array, data_f):
        tiempo_b= byte_array[len(byte_array)-8:len(byte_array)]
        tiempo_f=struct.unpack('d',tiempo_b)[0]
        data_f.append(tiempo_f)
        return data_f

def main(args=None):
    rclpy.init(args=args)
    node = ReceiveXbeeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()