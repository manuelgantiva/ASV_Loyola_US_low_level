
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from digi.xbee.devices import XBeeDevice
import struct
from asv_interfaces.msg import StateNeighbor

class XbeeTransceiverDSG(Node):
    def __init__(self):
        super().__init__("xbee_transceiver")
        self.declare_parameter("my_id", "ASV0")
        my_id = self.get_parameter("my_id").get_parameter_value().string_value
        try:
            self.xbee = XBeeDevice("/dev/xbee_usb", 115200)#<-----------------------------------------------------------------------------cambiar el puerto
            self.get_logger().info("\033[32mSerial Xbee port opened successfully...\033[0m")
        except Exception as e:
            self.get_logger().info("\033[31mSerial XBee port opening failure\033[0m")

        self.xbee.open()
        self.xbee.add_data_received_callback(self.callback_received_data) # Agregar el callback de recepción de datos

        self.publisher_ = self.create_publisher(StateNeighbor, "/" + my_id +"/reception/received_data", 10)
        self.subscriber_ = self.create_subscription(StateNeighbor, "/" + my_id +"/emission/sent_data", self.callback_ref, 10)

        self.get_logger().info("Xbee Transceiver Node in " + my_id + " has been started")

    def callback_ref(self, msg: StateNeighbor):

        byte_array = bytearray() # Crear arreglo de bytes
        
        data_list = [
            msg.point.x,
            msg.point.y,
            msg.point.z,
            msg.velocity.x,
            msg.velocity.y,
            msg.velocity.z]

        int_value = int(msg.id)

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

        self.publisher_.publish(info_rcv)


def main(args=None):
    rclpy.init(args=args)
    node = XbeeTransceiverDSG()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
