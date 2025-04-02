#!/usr/bin/env python3
import sys
import os
# 将当前脚本所在目录添加到 sys.path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
import time
from ros_robot_controller_sdk import Board


class IMUDisplayNode(Node):
    def __init__(self):
        super().__init__('imu_print_node')
        self.board = Board(device="/dev/ttyS1", baudrate=1000000) 
        self.board.enable_reception()
        self.get_logger().info("Board initialized")
        time.sleep(1)  # 等待1秒
        self.timer = self.create_timer(0.1, self.display_imu_callback)  # 每0.1秒读取一次

    def display_imu_callback(self):
        imu_data = self.board.get_imu()
        if imu_data is not None:
            ax, ay, az, gx, gy, gz = imu_data
            self.get_logger().info(f"IMU - Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) g, Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}) deg/s")


def main(args=None):
    rclpy.init(args=args)
    imu_print_node = IMUDisplayNode()
    rclpy.spin(imu_print_node)
    imu_print_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()