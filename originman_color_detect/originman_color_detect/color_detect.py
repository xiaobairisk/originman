#!/usr/bin/env python3
# encoding:utf-8
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import threading
import time
import math
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC

class ColorDetectNode(Node):
    def __init__(self):
        super().__init__('color_detect_node')
        self.bridge = CvBridge()
        
        # 创建图像发布者
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        # 创建颜色检测信息发布者
        self.color_info_pub = self.create_publisher(String, 'color_info', 10)
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return
        
        # 机器人控制初始化
        self.board = rrc.Board()
        self.ctl = Controller(self.board)
        self.servo_data = {'servo1': 1500, 'servo2': 1500}  # 默认舵机位置
        
        # 初始化舵机位置
        self.init_move()
        
        # 颜色检测相关参数
        self.size = (320, 240)
        self.color_list = []
        self.detect_color = 'None'
        self.action_finish = True
        self.draw_color = (0, 0, 0)  # 默认黑色
        
        # 颜色范围 (LAB 空间，需校准)
        self.lab_data = {
            'red': {'min': [0, 166, 135], 'max': [255, 255, 255]},
            'green': {'min': [47, 0, 135], 'max': [255, 110, 255]},
            'blue': {'min': [0, 0, 0], 'max': [255, 146, 120]}
        }
        
        # 颜色对应的 RGB 显示值
        self.range_rgb = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'black': (0, 0, 0)
        }
        
        # 定时器用于图像处理和发布
        self.timer = self.create_timer(0.1, self.process_image)
        
        # 启动动作控制线程
        self.action_thread = threading.Thread(target=self.move)
        self.action_thread.daemon = True
        self.action_thread.start()
        
        self.get_logger().info("颜色识别节点启动")

    def init_move(self):
        """初始化舵机位置"""
        self.ctl.set_pwm_servo_pulse(1, 1500, 500)
        self.ctl.set_pwm_servo_pulse(2, self.servo_data['servo2'], 500)
        self.get_logger().info("舵机初始化完成")

    def get_area_max_contour(self, contours):
        """找出面积最大的轮廓"""
        max_area = 0
        max_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > max_area and area > 50:  # 最小面积阈值
                max_area = area
                max_contour = c
        return max_contour, max_area

    def color_detect(self, img):
        """颜色检测"""
        img_h, img_w = img.shape[:2]
        
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        
        max_area = 0
        color_area_max = None
        area_max_contour_max = None
        center_x, center_y, radius = -1, -1, 0  # 初始化默认值
        
        if self.action_finish:
            for color_name in self.lab_data:
                mask = cv2.inRange(frame_lab,
                                  tuple(self.lab_data[color_name]['min']),
                                  tuple(self.lab_data[color_name]['max']))
                eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                area_max_contour, area_max = self.get_area_max_contour(contours)
                
                if area_max_contour is not None and area_max > max_area:
                    max_area = area_max
                    color_area_max = color_name
                    area_max_contour_max = area_max_contour
        
            if max_area > 200:  # 检测到有效目标
                (center_x_small, center_y_small), radius_small = cv2.minEnclosingCircle(area_max_contour_max)
                center_x = int(center_x_small * img_w / self.size[0])
                center_y = int(center_y_small * img_h / self.size[1])
                radius = int(radius_small * img_w / self.size[0])
                cv2.circle(img, (center_x, center_y), radius, self.range_rgb[color_area_max], 2)
                
                if color_area_max == 'red':
                    color = 1
                elif color_area_max == 'green':
                    color = 2
                elif color_area_max == 'blue':
                    color = 3
                else:
                    color = 0
                
                self.color_list.append(color)
                
                if len(self.color_list) == 3:  # 多次判断
                    color = int(round(np.mean(np.array(self.color_list))))
                    self.color_list = []
                    if color == 1:
                        self.detect_color = 'red'
                        self.draw_color = self.range_rgb["red"]
                    elif color == 2:
                        self.detect_color = 'green'
                        self.draw_color = self.range_rgb["green"]
                    elif color == 3:
                        self.detect_color = 'blue'
                        self.draw_color = self.range_rgb["blue"]
                    else:
                        self.detect_color = 'None'
                        self.draw_color = self.range_rgb["black"]
                    self.get_logger().info(f"识别到颜色: {self.detect_color}")
            else:
                self.detect_color = 'None'
                self.draw_color = self.range_rgb["black"]
        
        cv2.putText(img, "Color: " + self.detect_color, (10, img_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)
        
        return center_x, center_y, radius

    def move(self):
        """机器人动作控制线程"""
        while True:
            if rclpy.ok():
                if self.detect_color != 'None':
                    self.action_finish = False
                    if self.detect_color == 'red':
                        self.get_logger().info("检测到红色，执行点头动作")
                        self.ctl.set_pwm_servo_pulse(1, 1800, 200)
                        time.sleep(0.2)
                        self.ctl.set_pwm_servo_pulse(1, 1200, 200)
                        time.sleep(0.2)
                        self.ctl.set_pwm_servo_pulse(1, 1800, 200)
                        time.sleep(0.2)
                        self.ctl.set_pwm_servo_pulse(1, 1200, 200)
                        time.sleep(0.2)
                        self.ctl.set_pwm_servo_pulse(1, 1500, 100)
                        time.sleep(0.1)
                    elif self.detect_color in ['green', 'blue']:
                        self.get_logger().info("检测到绿色或蓝色，执行摇头动作")
                        self.ctl.set_pwm_servo_pulse(2, 1800, 200)
                        time.sleep(0.2)
                        self.ctl.set_pwm_servo_pulse(2, 1200, 200)
                        time.sleep(0.2)
                        self.ctl.set_pwm_servo_pulse(2, 1800, 200)
                        time.sleep(0.2)
                        self.ctl.set_pwm_servo_pulse(2, 1200, 200)
                        time.sleep(0.2)
                        self.ctl.set_pwm_servo_pulse(2, 1500, 100)
                        time.sleep(0.1)
                    
                    self.detect_color = 'None'
                    self.draw_color = self.range_rgb["black"]
                    time.sleep(1)
                    self.action_finish = True
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.01)

    def process_image(self):
        """处理图像并发布"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("无法读取图像帧")
            return
        
        # 颜色检测
        center_x, center_y, radius = self.color_detect(frame)
        color_msg = String()
        color_msg.data = f"Color: {self.detect_color}, X: {center_x}, Y: {center_y}, Radius: {radius}"
        self.color_info_pub.publish(color_msg)
        
        # 发布图像
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被中断")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()