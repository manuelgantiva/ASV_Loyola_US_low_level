#!/usr/bin/env python3
import math
import serial
import struct
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]


# 定义IMU驱动节点类
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


def handle_serial_data(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
    angle_flag = False
    buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    # According to the judgment of the data length bit, the corresponding length data can be obtained
    if key < 11:
        return
    else:
        data_buff = list(buff.values())  # Get dictionary ownership value
        if buff[1] == 0x51:
            if check_sum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
            else:
                print('0x51 Check failure')

        elif buff[1] == 0x52:
            if check_sum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in
                                   range(0, 3)]
            else:
                print('0x52 Check failure')

        elif buff[1] == 0x53:
            if check_sum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                angle_flag = True
            else:
                print('0x53 Check failure')
        elif buff[1] == 0x54:
            if check_sum(data_buff[0:10], data_buff[10]):
                magnetometer = hex_to_short(data_buff[2:10])
            else:
                print('0x54 Check failure')
        else:
            buff = {}
            key = 0

        buff = {}
        key = 0
        return angle_flag


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    return [qx, qy, qz, qw]


class IMUDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        self.declare_parameter("port", "/dev/imu_usb")
        self.declare_parameter("baud", 115200)

        self.imu_pub = self.create_publisher(Imu, 'imu_ext/data',qos_profile_sensor_data)
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud').value

        self.driver_thread = threading.Thread(target=self.driver_loop)
        self.driver_thread.start()

    def driver_loop(self):
        try:
            wt_imu = serial.Serial(port=self.port, baudrate=self.baud_rate, timeout=0.5)
            if wt_imu.isOpen():
                self.get_logger().info("\033[32mSerial Imu port opened successfully...\033[0m")
            else:
                wt_imu.open()
                self.get_logger().info("\033[32mSerial Imu port opened successfully...\033[0m")
        except Exception as e:
            print(e)
            self.get_logger().info("\033[31mSerial Imu port opening failure\033[0m")
            exit(0)
        while True:
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu disconnect")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count)
                    for i in range(0, buff_count):
                        tag = handle_serial_data(buff_data[i])
                        if tag:
                            self.imu_data()

    def imu_data(self):
        accel_x, accel_y, accel_z = acceleration[0], acceleration[1], acceleration[2]
        gyro_x, gyro_y, gyro_z = angularVelocity[0], angularVelocity[1], angularVelocity[
            2]
        gyro_scale = 2000 / 32768.0
        gyro_x, gyro_y, gyro_z = math.radians(gyro_x * gyro_scale), math.radians(gyro_y * gyro_scale), math.radians(
            gyro_z * gyro_scale)

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z
        self.imu_msg.angular_velocity.x = gyro_x
        self.imu_msg.angular_velocity.y = gyro_y
        self.imu_msg.angular_velocity.z = gyro_z

        angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]

        qua = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

        self.imu_msg.orientation.x = qua[0]
        self.imu_msg.orientation.y = qua[1]
        self.imu_msg.orientation.z = qua[2]
        self.imu_msg.orientation.w = qua[3]

        self.imu_pub.publish(self.imu_msg)

def main():
    rclpy.init()
    node = IMUDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
