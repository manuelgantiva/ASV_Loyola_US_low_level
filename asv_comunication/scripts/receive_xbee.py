#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from asv_interfaces.msg import AsvXbee

from digi.xbee.devices import XBeeDevice 
import time
import struct

class ReceiveXbeeNode(Node):
    def __init__(self):
        super().__init__("receive_node")
        device2=XBeeDevice("/dev/ttyUSB0",115200)
        device2.open() #Abrimos la comunicación
        device2.add_data_received_callback(self.my_data_received_callback)
        self.publisher_=self.create_publisher(AsvXbee, "asv_neighbor", 10)
        self.get_logger().info("Receive Xbee Node has been started")

    def my_data_received_callback(self, xbee_message):
        byte_array=xbee_message.data #Extraemos el dato del mensaje
        data_f = [] #Creamos la lista que contendrá los valores en flotante decodificados
        data_f= self.bytes2f(byte_array, data_f) #Ejecutamos la función que decodifica los bytes recogidos y los vuelve a convertir en punto flotante
        data_f= self.bytes2i(byte_array, data_f)
        data_f= self.bytes2d(byte_array, data_f)
        # data_f.append(time.time()-data_f[9])
        # data_f.append(time.time())
        info_rcv = AsvXbee()
        info_rcv.states.px = data_f[0]
        info_rcv.states.py = data_f[1]
        info_rcv.states.pz = data_f[2]
        info_rcv.states.vx = data_f[3]
        info_rcv.states.vy = data_f[4]
        info_rcv.states.vz = data_f[5]
        info_rcv.states.sx = data_f[6]
        info_rcv.states.sy = data_f[7]
        info_rcv.states.sz = data_f[8]
        info_rcv.id = data_f[9]
        info_rcv.time = data_f[10]
        self.publisher_.publish(info_rcv)



    def bytes2f(self, byte_array, data_f):
        for i in range(0, len(byte_array)-10, 4):
            numero_b = byte_array[i:i+4] #Vamos a ir desempaquetando cada número flotante por separado
            # Unpack the chunk as a single float value
            numero_f = struct.unpack('!f', numero_b)[0] #Unpack solo trabaja con 4 bytes a la vez, por eso la necesidad de dividir nuestro frame
            # Append the float value to the result list
            data_f.append(numero_f) #Vamos añadiendo cada numero decodificado a una lista
        return data_f

    def bytes2d(self, byte_array, data_f):
        tiempo_b= byte_array[len(byte_array)-8:]
        tiempo_f=struct.unpack('d',tiempo_b)[0]
        data_f.append(tiempo_f)
        return data_f

    def bytes2i(self,byte_array, data_f):
    	id_b=byte_array[len(byte_array)-10:len(byte_array)-8]
    	id_i=struct.unpack('h',id_b)[0]
    	data_f.append(id_i)
    	return data_f


def main(args=None):
    rclpy.init(args=args)
    node = ReceiveXbeeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
