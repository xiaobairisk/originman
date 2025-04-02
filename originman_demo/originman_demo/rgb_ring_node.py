#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import cv2
import random
import numpy as np
from smbus2 import SMBus, i2c_msg


class RGBRing:
    def __init__(self, pixel_num=5):
        self.bus = 5
        self.addr = 0x64

        self.pixels_num = pixel_num
        self.pixel_index = self.pixels_num - 1
        self.color_index = 0
        self.color_indexs = [30,] * self.pixels_num
        self.saturation = [255, ] * self.pixels_num
        self.values = [255,] * self.pixels_num
        self.values_inc = [5,] * self.pixels_num
        self.breate_rate = [2,]
        self.timestamp = time.time()

    def send_rgbs(self, rgbs):
        try:
            with SMBus(self.bus) as bus:
                bus.write_byte_data(self.addr, 0x00, self.pixels_num)
                bus.write_byte_data(self.addr, 0x01, 0)
                addr = i2c_msg.write(0x64, [0x30])
                data = i2c_msg.write(0x64, rgbs)
                bus.i2c_rdwr(addr, data)
                bus.write_byte_data(self.addr, 0x02, 0x01)
        except Exception as e:
            print(e)

    def mode1_update(self):
        self.color_index += 5 # 改变色调
        if self.color_index >= 180: # 色调之触顶
            self.color_index = self.color_index % 180 # 新的颜色值, 取模
        # 逐个计算灯的色调值，并设置灯泡颜色缓存，一圈布满整个0-180范围
        rgbs = []
        for i in range(self.pixels_num):
            # 每个灯泡的色调间隔就是 180 / 灯泡数, 灯泡间会因为挡板散射混色而颜色连续
            c_index = (self.color_index + int(180 / self.pixels_num) * i) % 180
            # 用 opencv 将 hsv 转换为 rgb
            rgb = cv2.cvtColor(np.array([[[c_index, 255, 255], ],], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
            rgbs.extend(rgb)
        self.send_rgbs(rgbs)
        self.timestamp=time.time()

    def mode2_update(self):
        color_inc = [4, 5, 3, 1, 2]
        for i in range(self.pixels_num):
            self.color_indexs[i] += random.randint(1, 6)
            if self.color_indexs[i] > random.randint(175, 180):
                self.color_indexs[i] = 0
        rgbs = []
        for i in range(self.pixels_num):
            buf = cv2.cvtColor(np.array([[[self.color_indexs[i], 255, 255]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
            rgbs.extend(buf)
        self.send_rgbs(rgbs)


    def mode3_update(self):
        if time.time() - self.timestamp > 0.15:
            rgbs = []
            colors = [[0xFF, 0x00, 0x00], [0x00, 0xFF, 0x00], [0x00, 0x00, 0xFF], [0xFF, 0xFF, 0x00], [0xFF, 0x00, 0xFF], [0x00, 0xFF, 0xFF]]
            for i in range(self.pixels_num):
                rgbs.extend(colors[random.randint(0, 5)])
            self.timestamp = time.time()
            self.send_rgbs(rgbs)

    def mode4_update(self):
        if self.color_index >= 180:
            self.color_index = 0
        self.color_index += 1
        buf = cv2.cvtColor(np.array([[[self.color_index, 255, 255]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
        rgbs = []
        for i in range(self.pixels_num):
            rgbs.extend(buf)
        self.send_rgbs(rgbs)


    def mode5_update(self):
        rgbs = []
        for i in range(self.pixels_num):
            self.values[i] += self.values_inc[i]
            if self.values[i] >= 255:
                self.values[i] = 255
                self.values_inc[i] = -self.values_inc[i]
            if self.values[i] <= 0:
                self.values[i] = 0
                self.values_inc[i] = -self.values_inc[i]
            buf = cv2.cvtColor(np.array([[[self.color_indexs[i], self.saturation[i], self.values[i]]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
            rgbs.extend(buf)
        self.send_rgbs(rgbs)

    def mode6_update(self):
        rgbs = []
        if time.time() - self.timestamp > 0.1:
            for i in range(self.pixels_num):
                buf = cv2.cvtColor(np.array([[[self.color_indexs[i], self.saturation[i], self.values[i]]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
                rgbs.extend(buf)
            self.timestamp = time.time()
            rgbs.extend(buf)
            self.send_rgbs(rgbs)

    def mode0_update(self):
        rgbs = []
        for i in range(self.pixels_num):
            rgbs.extend([0, 0, 0])
        self.send_rgbs(rgbs)


class RGBRingNode(Node):
    def __init__(self):
        super().__init__('rgb_ring_node')
        self.rgb_ring = RGBRing(6)
        r, g, b = 255, 255, 0
        buf = cv2.cvtColor(np.array([[[r, g, b], ], ], dtype=np.uint8), cv2.COLOR_RGB2HSV).reshape(3)
        self.rgb_ring.color_indexs = [buf[0],] * self.rgb_ring.pixels_num
        self.rgb_ring.saturation = [buf[1],] * self.rgb_ring.pixels_num
        self.rgb_ring.values = [buf[2],] * self.rgb_ring.pixels_num
        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        self.rgb_ring.mode1_update()


def main(args=None):
    rclpy.init(args=args)
    rgb_ring_node = RGBRingNode()
    rclpy.spin(rgb_ring_node)
    rgb_ring_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()