#!/usr/bin/env python3
import time
from ros_robot_controller_sdk import Board

def display_imu(board):
    """实时读取并显示IMU数据"""
    print("Starting IMU display (press Ctrl+C to stop)")
    try:
        while True:
            imu_data = board.get_imu()
            if imu_data is not None:
                ax, ay, az, gx, gy, gz = imu_data
                print(f"IMU - Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) g, Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}) deg/s")
            time.sleep(0.1)  # 每0.1秒读取一次
    except KeyboardInterrupt:
        print("IMU display stopped")


def main():
    # 初始化Board
    board = Board(device="/dev/ttyS1", baudrate=1000000)  # 根据实际串口调整
    board.enable_reception()
    print("Board initialized")

    time.sleep(1)       # 等待1秒
    display_imu(board)  # 显示IMU数据

if __name__ == "__main__":
    main()
