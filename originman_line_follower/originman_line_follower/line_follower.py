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

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('visual_patrol_node')
        self.bridge = CvBridge()
        
        # 创建图像发布者
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        # 创建巡线信息发布者
        self.line_info_pub = self.create_publisher(String, 'line_info', 10)
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return
        
        # 机器人控制初始化
        self.board = rrc.Board()
        self.ctl = Controller(self.board)
        self.servo_data = {'servo1': 891, 'servo2': 1661}
        
        # 初始化舵机位置
        self.init_move()
        
        # 巡线相关参数
        self.target_color = 'black'  # 默认追踪黑色线
        self.size = (640, 480)
        self.img_centerx = self.size[0] / 2
        self.line_center_x = -1
        
        # ROI 和权重
        self.roi = [
            (240, 280, 0, 640, 0.1),  # [ROI, weight]
            (340, 380, 0, 640, 0.3),
            (440, 480, 0, 640, 0.6)
        ]
        self.roi_h1 = self.roi[0][0]
        self.roi_h2 = self.roi[1][0] - self.roi[0][0]
        self.roi_h3 = self.roi[2][0] - self.roi[1][0]
        self.roi_h_list = [self.roi_h1, self.roi_h2, self.roi_h3]
        
        # 颜色范围
        self.lab_data = {
            'black': {'min': [0, 166, 135], 'max': [255, 255, 255]} 
        } 
        
        
        # 定时器用于图像处理和发布
        self.timer = self.create_timer(0.1, self.process_image)
        
        # 启动动作控制线程
        self.action_thread = threading.Thread(target=self.move)
        self.action_thread.daemon = True
        self.action_thread.start()
        
        self.get_logger().info("视觉巡线节点启动，目标颜色: {}".format(self.target_color))

    def init_move(self):
        """初始化舵机位置"""
        self.ctl.set_pwm_servo_pulse(1, self.servo_data['servo1'], 500)
        self.ctl.set_pwm_servo_pulse(2, self.servo_data['servo2'], 500)
        self.get_logger().info("舵机初始化完成")

    def get_area_max_contour(self, contours):
        """找出面积最大的轮廓"""
        max_area = 0
        max_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > max_area and area > 5:  # 最小面积阈值
                max_area = area
                max_contour = c
        return max_contour, max_area

    def line_detect(self, img):
        """巡线检测"""
        img_h, img_w = img.shape[:2]
        
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        
        centroid_x_sum = 0
        weight_sum = 0
        center_ = []
        n = 0

        for r in self.roi:
            roi_h = self.roi_h_list[n]
            n += 1       
            blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
            frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)
            
            mask = cv2.inRange(frame_lab,
                              tuple(self.lab_data[self.target_color]['min']),
                              tuple(self.lab_data[self.target_color]['max']))
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated[:, 0:160] = 0  # 屏蔽左侧
            dilated[:, 480:640] = 0  # 屏蔽右侧
            cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
            cnt_large, area = self.get_area_max_contour(cnts)
            
            if cnt_large is not None:
                rect = cv2.minAreaRect(cnt_large)
                box = np.int0(cv2.boxPoints(rect))
                for i in range(4):
                    box[i, 1] = box[i, 1] + (n - 1) * roi_h + self.roi[0][0]
                    box[i, 1] = int(box[i, 1] * img_h / self.size[1])
                    box[i, 0] = int(box[i, 0] * img_w / self.size[0])
                
                cv2.drawContours(img, [box], -1, (0, 0, 255, 255), 2)
                
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
                cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                
                center_.append([center_x, center_y])
                centroid_x_sum += center_x * r[4]
                weight_sum += r[4]
        
        if weight_sum != 0:
            self.line_center_x = int(centroid_x_sum / weight_sum)
            cv2.circle(img, (self.line_center_x, int(center_y)), 10, (0, 255, 255), -1)
            self.get_logger().info(f"检测到黑线中心: {self.line_center_x}")
        else:
            self.line_center_x = -1
        
        return img

    def move(self):
        """机器人动作控制线程"""
        while True:
            if rclpy.ok():
                if self.line_center_x != -1:
                    if abs(self.line_center_x - self.img_centerx) <= 50:
                        AGC.runActionGroup('go_forward')
                        self.get_logger().info("沿黑线前进")
                    elif self.line_center_x - self.img_centerx > 50:
                        AGC.runActionGroup('turn_right_small_step')
                        self.get_logger().info("向右调整")
                    elif self.line_center_x - self.img_centerx < -50:
                        AGC.runActionGroup('turn_left_small_step')
                        self.get_logger().info("向左调整")
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
        
        # 巡线检测
        frame = self.line_detect(frame)
        line_msg = String()
        line_msg.data = f"Line Center X: {self.line_center_x}"
        self.line_info_pub.publish(line_msg)
        
        # 发布图像
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被中断")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()