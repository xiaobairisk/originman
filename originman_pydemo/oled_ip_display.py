#!/usr/bin/env python3
import time
import numpy as np
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont
import socket
import subprocess

class OledDisplayNode:
    def __init__(self):
        # 初始化OLED显示屏
        self.screen = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=5, gpio=1, i2c_address=0x3C)
        self.screen.begin()  # 启动屏幕
        self.screen.image(Image.fromarray(np.zeros((64, 128), dtype=np.uint8)).convert('1'))
        self.screen.display()

        # 加载字体
        self.font = ImageFont.load_default()

        # 主循环，定时更新显示内容
        while True:
            try:
                self.update()
                self.screen.image(Image.fromarray(self.gram).convert('1'))
                self.screen.display()
                time.sleep(1)  # 每秒刷新一次
            except KeyboardInterrupt:
                # 退出时清空屏幕
                img = Image.new('1', (self.screen.width, self.screen.height), color=0)
                self.gram = np.array(img, dtype=np.uint8) * 255
                self.screen.image(Image.fromarray(self.gram).convert('1'))
                self.screen.display()
                break

    def get_wlan0_ip(self):
        """获取wlan0的IP地址，如果未连接则返回None"""
        try:
            # 使用ifconfig获取wlan0的IP地址
            result = subprocess.check_output(["ifconfig", "wlan0"], text=True)
            for line in result.split('\n'):
                if "inet " in line:
                    ip = line.split('inet ')[1].split(' ')[0]  # 提取IP地址部分
                    return ip
            return None  # 如果没有inet行，说明未连接
        except subprocess.CalledProcessError:
            # 如果ifconfig wlan0失败（例如接口不存在或未启用）
            return None

    def update(self):
        """更新OLED显示内容"""
        # 创建空白图像
        img = Image.new('1', (self.screen.width, self.screen.height), color=0)
        draw = ImageDraw.Draw(img)

        # 获取wlan0的IP地址
        ip_address = self.get_wlan0_ip()

        if ip_address:
            # 如果有IP地址，显示"IP: xxx.xxx.xxx.xxx"
            display_text = f"IP: {ip_address}"
            draw.text((3, 22), display_text, font=self.font, fill=255)
        else:
            # 如果未连接网络，显示"网络未连接"
            draw.text((3, 22), "Network not connected", font=self.font, fill=255)

        # 将图像转换为numpy数组并更新self.gram
        self.gram = np.array(img, dtype=np.uint8) * 255

if __name__ == "__main__":
    OledDisplayNode()
