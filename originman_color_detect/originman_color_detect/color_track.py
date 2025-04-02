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
import hiwonder.PID as PID

class ColorTrackNode(Node):
    def __init__(self):
        super().__init__('color_track_node')
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
        self.servo_data = {'servo1': 891, 'servo2': 1661}
        self.x_dis = self.servo_data['servo2']  # 水平方向初始位置
        self.y_dis = self.servo_data['servo1']  # 垂直方向初始位置
        
        # PID 初始化
        self.x_pid = PID.PID(P=0.145, I=0.00, D=0.0007)
        self.y_pid = PID.PID(P=0.145, I=0.00, D=0.0007)
        
        # 颜色检测相关参数
        self.target_color = 'red'  # 默认追踪红色
        self.size = (320, 240)
        self.center_x = self.size[0] / 2
        self.center_y = self.size[1] / 2
        
        # 颜色范围
        self.lab_data = {
            'red': {'min': [0, 166, 135], 'max': [255, 255, 255]},
            'green': {'min': [47, 0, 135], 'max': [255, 110, 255]},
            'blue': {'min': [0, 0, 0], 'max': [255, 146, 120]}
        }
        
        # 颜色对应的 RGB 显示值
        self.range_rgb = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0)
        }
        
        # 初始化舵机位置
        self.init_move()
        
        # 定时器用于图像处理和发布
        self.timer = self.create_timer(0.1, self.process_image)
        
        # 启动舵机控制线程
        self.servo_thread = threading.Thread(target=self.servo_control)
        self.servo_thread.daemon = True
        self.servo_thread.start()
        
        self.get_logger().info("颜色追踪节点启动，目标颜色: {}".format(self.target_color))

    def init_move(self):
        """初始化舵机位置"""
        self.ctl.set_pwm_servo_pulse(1, self.y_dis, 500)
        self.ctl.set_pwm_servo_pulse(2, self.x_dis, 500)
        self.get_logger().info("舵机初始化完成")

    def get_area_max_contour(self, contours):
        """找出面积最大的轮廓"""
        max_area = 0
        max_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > max_area and area > 10:  # 最小面积阈值
                max_area = area
                max_contour = c
        return max_contour, max_area

    def color_detect(self, img):
        """颜色检测并追踪"""
        img_h, img_w = img.shape[:2]
        
        # 绘制中心十字线
        cv2.line(img, (int(img_w/2 - 10), int(img_h/2)), (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
        cv2.line(img, (int(img_w/2), int(img_h/2 - 10)), (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)
        
        # 图像预处理
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        
        # 颜色检测
        mask = cv2.inRange(frame_lab,
                          tuple(self.lab_data[self.target_color]['min']),
                          tuple(self.lab_data[self.target_color]['max']))
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        area_max_contour, area_max = self.get_area_max_contour(contours)
        
        center_x, center_y, radius = -1, -1, 0
        if area_max > 20:  # 检测到有效目标
            (center_x_small, center_y_small), radius_small = cv2.minEnclosingCircle(area_max_contour)
            # 替换 Misc.map，直接计算比例
            center_x = int(center_x_small * img_w / self.size[0])
            center_y = int(center_y_small * img_h / self.size[1])
            radius = int(radius_small * img_w / self.size[0])
            cv2.circle(img, (center_x, center_y), radius, self.range_rgb[self.target_color], 2)
            
            # PID 控制
            if abs(center_x - img_w / 2.0) < 20:
                center_x = img_w / 2.0
            self.x_pid.SetPoint = img_w / 2
            self.x_pid.update(center_x)
            dx = int(self.x_pid.output)
            use_time = abs(dx * 0.00025)
            self.x_dis += dx
            self.x_dis = max(500, min(2500, self.x_dis))  # 限制范围
            
            if abs(center_y - img_h / 2.0) < 20:
                center_y = img_h / 2.0
            self.y_pid.SetPoint = img_h / 2
            self.y_pid.update(center_y)
            dy = int(self.y_pid.output)
            use_time = round(max(use_time, abs(dy * 0.00025)), 5)
            self.y_dis += dy
            self.y_dis = max(1000, min(2000, self.y_dis))  # 限制范围
            
            self.get_logger().info(f"检测到 {self.target_color}, 中心: ({center_x}, {center_y}), 舵机位置: (x: {self.x_dis}, y: {self.y_dis})")
        
        return center_x, center_y, radius

    def servo_control(self):
        """舵机控制线程"""
        while True:
            if rclpy.ok():
                use_time = 0.05  # 默认移动时间
                self.ctl.set_pwm_servo_pulse(1, self.y_dis, int(use_time * 1000))
                self.ctl.set_pwm_servo_pulse(2, self.x_dis, int(use_time * 1000))
                time.sleep(use_time)
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
        color_msg.data = f"Color: {self.target_color}, X: {center_x}, Y: {center_y}, Radius: {radius}"
        self.color_info_pub.publish(color_msg)
        
        # 发布图像
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被中断")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()