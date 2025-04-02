#!/usr/bin/env python3
# encoding:utf-8
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import apriltag
import numpy as np
import threading
import time
import math
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC

class TransportNode(Node):
    def __init__(self):
        super().__init__('transport_node')
        self.bridge = CvBridge()
        
        # 创建图像发布者
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        # 创建 AprilTag 和颜色检测信息发布者
        self.tag_info_pub = self.create_publisher(String, 'apriltag_info', 10)
        self.color_info_pub = self.create_publisher(String, 'color_info', 10)
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return
        
        # 初始化 AprilTag 检测器
        self.detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
        
        # 机器人控制初始化
        self.board = rrc.Board()
        self.ctl = Controller(self.board)
        self.servo_data = {'servo1': 891, 'servo2': 1661}
        self.x_dis = self.servo_data['servo2']  # 水平方向初始位置
        self.y_dis = self.servo_data['servo1']  # 垂直方向初始位置
        
        # 搬运相关参数
        self.size = (320, 240)
        self.CENTER_X = 350
        self.d_x = 15
        self.d_y = 15
        self.step = 1
        self.time_start = 0
        self.last_status = ''
        self.start_count = True
        self.object_center_x = -2
        self.object_center_y = -2
        self.object_angle = 0
        self.turn = 'None'
        self.find_box = True
        self.go_step = 3
        self.lock_servos = ''
        self.stop_detect = False
        self.object_color = 'red'
        self.haved_find_tag = False
        self.head_turn = 'left_right'
        self.color_list = ['red', 'green', 'blue']
        self.color_center_x = -1
        self.color_center_y = -1
        
        # 颜色范围 (LAB 空间，需校准)
        self.lab_data = {
            'red': {'min': [0, 166, 135], 'max': [255, 255, 255]},
            'green': {'min': [47, 0, 135], 'max': [255, 110, 255]},
            'blue': {'min': [0, 0, 0], 'max': [255, 146, 120]}
        }
        
        # 颜色对应的 RGB 显示值和 AprilTag ID
        self.range_rgb = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255)
        }
        self.color_tag = {'red': 1, 'green': 2, 'blue': 3}
        
        # 动作组名称
        self.go_forward = 'go_forward'
        self.back = 'back_fast'
        self.turn_left = 'turn_left_small_step'
        self.turn_right = 'turn_right_small_step'
        self.left_move = 'left_move'
        self.right_move = 'right_move'
        self.left_move_large = 'left_move_30'
        self.right_move_large = 'right_move_30'
        
        # 舵机锁定值
        self.LOCK_SERVOS = {'6': 650, '7': 850, '8': 0, '14': 350, '15': 150, '16': 1000}
        
        # 初始化舵机位置
        self.init_move()
        
        # 定时器用于图像处理和发布
        self.timer = self.create_timer(0.1, self.process_image)
        
        # 启动动作控制线程
        self.action_thread = threading.Thread(target=self.move)
        self.action_thread.daemon = True
        self.action_thread.start()
        
        self.get_logger().info("智能搬运节点启动")

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
            if area > max_area and area > 10:
                max_area = area
                max_contour = c
        return max_contour, max_area

    def color_detect(self, img):
        """红绿蓝颜色检测"""
        img_h, img_w = img.shape[:2]
        
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        
        center_max_distance = pow(img_w/2, 2) + pow(img_h, 2)
        color, center_x, center_y, angle = 'None', -1, -1, 0
        for i in self.color_list:
            mask = cv2.inRange(frame_lab,
                              tuple(self.lab_data[i]['min']),
                              tuple(self.lab_data[i]['max']))
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            area_max_contour, area_max = self.get_area_max_contour(contours)
            
            if area_max > 500:
                rect = cv2.minAreaRect(area_max_contour)
                angle_ = rect[2]
                box = np.int0(cv2.boxPoints(rect))
                for j in range(4):
                    box[j, 0] = int(box[j, 0] * img_w / self.size[0])
                    box[j, 1] = int(box[j, 1] * img_h / self.size[1])
                
                cv2.drawContours(img, [box], -1, (0, 255, 255), 2)
                
                ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x_, center_y_ = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)
                cv2.circle(img, (center_x_, center_y_), 5, (0, 255, 255), -1)
                
                distance = pow(center_x_ - img_w/2, 2) + pow(center_y_ - img_h, 2)
                if distance < center_max_distance:
                    center_max_distance = distance
                    color = i
                    center_x, center_y, angle = center_x_, center_y_, angle_
                    self.get_logger().info(f"检测到颜色目标: {color}, 位置: ({center_x}, {center_y}), 角度: {angle}")
        
        return color, center_x, center_y, angle

    def apriltag_detect(self, img):
        """AprilTag 检测"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray, return_image=False)
        tag_data = [[-1, -1, 0], [-1, -1, 0], [-1, -1, 0]]  # tag_1, tag_2, tag_3
        
        if detections:
            for detection in detections:
                corners = np.rint(detection.corners).astype(int)
                cv2.drawContours(img, [corners], -1, (0, 255, 255), 2)
                
                tag_family = str(detection.tag_family, encoding='utf-8')
                tag_id = str(detection.tag_id)
                object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])
                cv2.circle(img, (object_center_x, object_center_y), 5, (0, 255, 255), -1)
                
                object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))
                
                if tag_family == 'tag36h11':
                    if tag_id == '1':
                        tag_data[0] = [object_center_x, object_center_y, object_angle]
                    elif tag_id == '2':
                        tag_data[1] = [object_center_x, object_center_y, object_angle]
                    elif tag_id == '3':
                        tag_data[2] = [object_center_x, object_center_y, object_angle]
                    self.get_logger().info(f"检测到 AprilTag ID: {tag_id}, 位置: ({object_center_x}, {object_center_y}), 角度: {object_angle}")
        
        return tag_data

    def get_turn(self, tag_id, tag_data):
        """通过其他 AprilTag 判断目标位置"""
        tag_1, tag_2, tag_3 = tag_data
        if tag_id == 1:
            if tag_2[0] == -1 and tag_3[0] != -1:
                return 'left'
            elif tag_2[0] != -1:
                return 'left'
        elif tag_id == 2:
            if tag_1[0] == -1 and tag_3[0] != -1:
                return 'left'
            elif tag_1[0] != -1:
                return 'right'
        elif tag_id == 3:
            if tag_1[0] == -1 and tag_2[0] != -1:
                return 'right'
            elif tag_1[0] != -1:
                return 'right'
        return 'None'

    def move(self):
        """机器人移动逻辑"""
        while True:
            if rclpy.ok():
                stage = "搬运阶段" if self.find_box else "放置阶段"
                target = self.object_color if self.find_box else f"AprilTag ID {self.color_tag[self.object_color]}"
                
                if self.object_center_x == -3:  # 放置阶段，未找到目标但找到其他 AprilTag
                    if self.turn == 'left':
                        AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)
                        self.get_logger().info(f"执行动作: {self.turn_left}")
                    elif self.turn == 'right':
                        AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
                        self.get_logger().info(f"执行动作: {self.turn_right}")
                
                elif self.haved_find_tag and self.object_center_x == -1:  # 转头找到 AprilTag，头回中时不在视野
                    if self.x_dis > self.servo_data['servo2']:
                        AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)
                        self.get_logger().info(f"执行动作: {self.turn_left}")
                    elif self.x_dis < self.servo_data['servo2']:
                        AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
                        self.get_logger().info(f"执行动作: {self.turn_right}")
                    else:
                        self.haved_find_tag = False
                
                elif self.object_center_x >= 0:  # 找到目标
                    self.get_logger().info(f"当前阶段: {stage}, 目标: {target}, 步骤: {self.step}")
                    
                    if not self.find_box and self.color_center_y > 350:
                        if (self.color_center_x - self.CENTER_X) > 80:
                            AGC.runActionGroup(self.go_forward, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.go_forward}")
                        elif (self.color_center_x > self.CENTER_X and self.object_center_x >= self.CENTER_X) or (self.color_center_x <= self.CENTER_X and self.object_center_x >= self.CENTER_X):
                            AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.right_move_large}")
                        elif (self.color_center_x > self.CENTER_X and self.object_center_x < self.CENTER_X) or (self.color_center_x <= self.CENTER_X and self.object_center_x < self.CENTER_X):
                            AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.left_move_large}")
                    
                    if self.x_dis != self.servo_data['servo2'] and not self.haved_find_tag:
                        self.head_turn = 'left_right'
                        self.start_count = True
                        self.d_x, self.d_y = 15, 15
                        self.haved_find_tag = True
                        self.ctl.set_pwm_servo_pulse(1, self.servo_data['servo1'], 500)
                        self.ctl.set_pwm_servo_pulse(2, self.servo_data['servo2'], 500)
                        time.sleep(0.6)
                        self.get_logger().info("头回中")
                    
                    elif self.step == 1:
                        self.x_dis = self.servo_data['servo2']
                        self.y_dis = self.servo_data['servo1']
                        self.turn = ''
                        self.haved_find_tag = False
                        if (self.object_center_x - self.CENTER_X) > 170 and self.object_center_y > 330:
                            AGC.runActionGroup(self.back, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.back}")
                        elif self.object_center_x - self.CENTER_X > 80:
                            AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.turn_right}")
                        elif self.object_center_x - self.CENTER_X < -80:
                            AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.turn_left}")
                        elif 0 < self.object_center_y <= 250:
                            AGC.runActionGroup(self.go_forward, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.go_forward}")
                        else:
                            self.step = 2
                            self.get_logger().info("调整至中心完成，进入步骤 2")
                    
                    elif self.step == 2:
                        if 330 < self.object_center_y:
                            AGC.runActionGroup(self.back, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.back}")
                        if self.find_box:
                            if self.object_center_x - self.CENTER_X > 150:
                                AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.right_move_large}")
                            elif self.object_center_x - self.CENTER_X < -150:
                                AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.left_move_large}")
                            elif -10 > self.object_angle > -45:
                                AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.turn_left}")
                            elif -80 < self.object_angle <= -45:
                                AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.turn_right}")
                            elif self.object_center_x - self.CENTER_X > 40:
                                AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.right_move_large}")
                            elif self.object_center_x - self.CENTER_X < -40:
                                AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.left_move_large}")
                            else:
                                self.step = 3
                                self.get_logger().info("接近物体完成，进入步骤 3")
                        else:
                            if self.object_center_x - self.CENTER_X > 150:
                                AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.right_move_large}")
                            elif self.object_center_x - self.CENTER_X < -150:
                                AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.left_move_large}")
                            elif self.object_angle < -5:
                                AGC.runActionGroup(self.turn_left, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.turn_left}")
                            elif 5 < self.object_angle:
                                AGC.runActionGroup(self.turn_right, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.turn_right}")
                            elif self.object_center_x - self.CENTER_X > 40:
                                AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.right_move_large}")
                            elif self.object_center_x - self.CENTER_X < -40:
                                AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.left_move_large}")
                            else:
                                self.step = 3
                                self.get_logger().info("接近物体完成，进入步骤 3")
                    
                    elif self.step == 3:
                        if 340 < self.object_center_y:
                            AGC.runActionGroup(self.back, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.back}")
                        elif 0 < self.object_center_y <= 250:
                            AGC.runActionGroup(self.go_forward, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.go_forward}")
                        elif self.object_center_x - self.CENTER_X >= 40:
                            AGC.runActionGroup(self.right_move_large, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.right_move_large}")
                        elif self.object_center_x - self.CENTER_X <= -40:
                            AGC.runActionGroup(self.left_move_large, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.left_move_large}")
                        elif 20 <= self.object_center_x - self.CENTER_X < 40:
                            AGC.runActionGroup(self.right_move, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.right_move}")
                        elif -40 < self.object_center_x - self.CENTER_X < -20:
                            AGC.runActionGroup(self.left_move, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.left_move}")
                        else:
                            self.step = 4
                            self.get_logger().info("精调位置完成，进入步骤 4")
                    
                    elif self.step == 4:
                        if 280 < self.object_center_y <= 340:
                            AGC.runActionGroup('go_forward_one_step', lock_servos=self.lock_servos)
                            time.sleep(0.2)
                            self.get_logger().info("执行动作: go_forward_one_step")
                        elif 0 <= self.object_center_y <= 280:
                            AGC.runActionGroup(self.go_forward, lock_servos=self.lock_servos)
                            self.get_logger().info(f"执行动作: {self.go_forward}")
                        else:
                            if self.object_center_y >= 370:
                                self.go_step = 2
                            else:
                                self.go_step = 3
                            if abs(self.object_center_x - self.CENTER_X) <= 40:
                                self.stop_detect = True
                                self.step = 5
                                self.get_logger().info("进入步骤 5")
                            else:
                                self.step = 3
                    
                    elif self.step == 5:
                        if self.find_box:
                            AGC.runActionGroup('go_forward_one_step', times=2)
                            AGC.runActionGroup('stand', lock_servos=self.lock_servos)
                            AGC.runActionGroup('move_up')
                            self.lock_servos = self.LOCK_SERVOS
                            self.step = 6
                            self.get_logger().info("拿起物体完成，进入步骤 6")
                        else:
                            AGC.runActionGroup('go_forward_one_step', times=self.go_step, lock_servos=self.lock_servos)
                            AGC.runActionGroup('stand', lock_servos=self.lock_servos)
                            AGC.runActionGroup('put_down')
                            AGC.runActionGroup(self.back, times=5, with_stand=True)
                            self.color_list.remove(self.object_color)
                            if not self.color_list:
                                self.color_list = ['red', 'green', 'blue']
                            self.lock_servos = ''
                            self.step = 6
                            self.get_logger().info("放下物体完成，进入步骤 6")
                
                elif self.object_center_x == -1:  # 未检测到目标，转头寻找
                    if self.start_count:
                        self.start_count = False
                        self.time_start = time.time()
                    else:
                        if time.time() - self.time_start > 0.5:
                            if 0 < self.servo_data['servo2'] - self.x_dis <= abs(self.d_x) and self.d_y > 0:
                                self.x_dis = self.servo_data['servo2']
                                self.y_dis = self.servo_data['servo1']
                                self.ctl.set_pwm_servo_pulse(1, self.y_dis, 20)
                                self.ctl.set_pwm_servo_pulse(2, self.x_dis, 20)
                                AGC.runActionGroup(self.turn_right, times=3, lock_servos=self.lock_servos)
                                self.get_logger().info(f"执行动作: {self.turn_right} (3次)")
                            elif self.head_turn == 'left_right':
                                self.x_dis += self.d_x
                                if self.x_dis > self.servo_data['servo2'] + 400 or self.x_dis < self.servo_data['servo2'] - 200:
                                    if self.head_turn == 'left_right':
                                        self.head_turn = 'up_down'
                                    self.d_x = -self.d_x
                            elif self.head_turn == 'up_down':
                                self.y_dis += self.d_y
                                if self.y_dis > self.servo_data['servo1'] + 300 or self.y_dis < self.servo_data['servo1']:
                                    if self.head_turn == 'up_down':
                                        self.head_turn = 'left_right'
                                    self.d_y = -self.d_y
                            self.ctl.set_pwm_servo_pulse(1, self.y_dis, 20)
                            self.ctl.set_pwm_servo_pulse(2, self.x_dis, 20)
                            time.sleep(0.02)
                            self.get_logger().info(f"搜索目标，舵机位置: (x: {self.x_dis}, y: {self.y_dis})")
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
        
        if self.stop_detect:
            if self.step == 5:
                self.object_center_x = 0
            elif self.step == 6:
                self.find_box = not self.find_box
                self.object_center_x = -2
                self.step = 1
                self.stop_detect = False
        else:
            color, self.color_center_x, self.color_center_y, color_angle = self.color_detect(frame)
            color_msg = String()
            color_msg.data = f"Color: {color}, X: {self.color_center_x}, Y: {self.color_center_y}, Angle: {color_angle}"
            self.color_info_pub.publish(color_msg)
            
            if self.find_box:
                self.object_color, self.object_center_x, self.object_center_y, self.object_angle = color, self.color_center_x, self.color_center_y, color_angle
            else:
                tag_data = self.apriltag_detect(frame)
                tag_msg = String()
                tag_msg.data = f"Tag 1: {tag_data[0]}, Tag 2: {tag_data[1]}, Tag 3: {tag_data[2]}"
                self.tag_info_pub.publish(tag_msg)
                
                if tag_data[self.color_tag[self.object_color] - 1][0] != -1:
                    self.object_center_x, self.object_center_y, self.object_angle = tag_data[self.color_tag[self.object_color] - 1]
                else:
                    self.turn = self.get_turn(self.color_tag[self.object_color], tag_data)
                    if self.turn == 'None':
                        self.object_center_x, self.object_center_y, self.object_angle = -1, -1, 0
                    else:
                        self.object_center_x, self.object_center_y, self.object_angle = -3, -1, 0
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = TransportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被中断")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()