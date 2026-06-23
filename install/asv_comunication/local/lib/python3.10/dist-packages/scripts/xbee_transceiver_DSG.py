#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from digi.xbee.devices import XBeeDevice
import struct
from asv_interfaces.msg import StateObserver, StateNeighbor
from collections import deque

class XbeeTransceiverDSG(Node):
    def __init__(self):
        super().__init__("xbee_transceiver")
        self.declare_parameter("my_id", "ASV0")
        self.declare_parameter("worker_mode", 0)  # 0: Master, 1, 2, ..: Slave -1:Bidir

        self.my_string_id = self.get_parameter("my_id").get_parameter_value().string_value
        self.worker_mode = self.get_parameter("worker_mode").get_parameter_value().integer_value
        try:
            if self.my_string_id[3] == "1":
                usb_tty = 0
            elif self.my_string_id[3] == "3":
                usb_tty = 1
            elif self.my_string_id[3] == "4":
                usb_tty = 2
            self.xbee = XBeeDevice(f"/dev/xbee_usb", 115200)#<-----------------------------------------------------------------------------cambiar el puerto
            self.get_logger().info("\033[32mSerial Xbee port opened successfully...\033[0m")
        except Exception as e:
            self.get_logger().info("Serial Sim port opening failure !!!!!!!!!!!!!!!!")


        self.xbee.open()
        self.xbee.add_data_received_callback(self.callback_received_data) # Agregar el callback de recepción de datos

        # por ahora solo -1
        self.states = deque([])
        self.subscriber_state = self.create_subscription(StateObserver, "/" + self.my_string_id +"/observer/state_observer",
                        self.callback_state_observer,qos_profile_sensor_data)
        self.publisher_hlc = self.create_publisher(StateNeighbor, "/" + self.my_string_id +"/neighbors/state_observer", 1)
        self.timer_ = self.create_timer(0.1, self.publish_incoming_msgs)
        
        self.get_logger().info("Xbee Transceiver Node in " + self.my_string_id + " has been started")

    def callback_state_observer(self, msg: StateObserver):

        byte_array = bytearray() # Crear arreglo de bytes
        
        data_list = [
            msg.point.x,
            msg.point.y,
            msg.point.z,
            msg.velocity.x,
            msg.velocity.y,
            msg.velocity.z]

        int_value = int(self.my_string_id[3])

        byte_array = struct.pack('!6fI', *data_list, int_value) # Paquete 6 floats + 1 int

        try:
            self.xbee.send_data_broadcast(byte_array) # Send data
        except Exception as e:
            self.get_logger().info("Data was not sent") # Catches exception and returns unsuccessful

    def callback_received_data(self, xbee_message):
        byte_array = xbee_message.data # Extraemos el dato del mensaje
        *data_f, id_ASV = struct.unpack('!6fI', byte_array) # Desempaquetamos los datos

        data_f = list(data_f)

        info_rcv = StateNeighbor()
        info_rcv.point.x = data_f[0]
        info_rcv.point.y = data_f[1]
        info_rcv.point.z = data_f[2]
        info_rcv.velocity.x = data_f[3]
        info_rcv.velocity.y = data_f[4]
        info_rcv.velocity.z = data_f[5]

        info_rcv.id = "ASV" + str(id_ASV)
        info_rcv.msg_from = id_ASV # distributed, for centralized will be worker_mode

        self.states.append(info_rcv)


    def publish_incoming_msgs(self):
        if self.worker_mode == -1:
            while(len(self.states) > 0):
                msg = self.states.popleft()
                self.publisher_hlc.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = XbeeTransceiverDSG()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
