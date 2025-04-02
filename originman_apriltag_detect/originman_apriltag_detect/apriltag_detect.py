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

class AprilTagDetectNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.bridge = CvBridge()
        # 创建发布者，发布 image_raw 话题
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        # 创建发布者，发布 AprilTag 识别信息
        self.apriltag_info_pub = self.create_publisher(String, 'apriltag_info', 10)
        # 打开摄像头，0 表示默认摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
        else:
            # 创建定时器，定期发布图像
            self.timer = self.create_timer(0.1, self.publish_image)
            # 用于标记是否为首次发布图像
            self.is_first_publish = True

        # 初始化 AprilTag 检测器
        self.detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                # 进行 AprilTag 检测
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detections = self.detector.detect(gray, return_image=False)

                apriltag_info_msg = String()
                if len(detections) != 0:
                    tag_info_list = []
                    for detection in detections:
                        corners = np.int0(detection.corners)  # 获取四个角点
                        cv2.drawContours(frame, [np.array(corners, int)], -1, (0, 255, 255), 2)

                        tag_family = str(detection.tag_family, encoding='utf-8')  # 获取 tag_family
                        tag_id = int(detection.tag_id)  # 获取 tag_id

                        object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点

                        object_angle = int(np.degrees(np.arctan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角

                        self.get_logger().info(f"检测到 AprilTag - ID: {tag_id}, 家族: {tag_family}")

                        # 在图像上绘制标签信息
                        cv2.putText(frame, f"tag_id: {tag_id}", (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
                        cv2.putText(frame, f"tag_family: {tag_family}", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)

                        tag_info_list.append(f"ID: {tag_id}, 家族: {tag_family}")

                    apriltag_info_msg.data = "; ".join(tag_info_list)
                else:
                    cv2.putText(frame, "tag_id: None", (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
                    cv2.putText(frame, "tag_family: None", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
                    apriltag_info_msg.data = "未检测到 AprilTag"

                # 发布 AprilTag 识别信息
                self.apriltag_info_pub.publish(apriltag_info_msg)

                # 将 OpenCV 图像转换为 ROS 图像消息
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                # 发布图像消息
                self.publisher_.publish(ros_image)
                if self.is_first_publish:
                    self.get_logger().info('正在检测AprilTag信息')
                    self.is_first_publish = False
            except Exception as e:
                self.get_logger().error(f"Error converting image: {str(e)}")
        else:
            self.get_logger().error("无法读取图像帧")

    def __del__(self):
        # 释放摄像头资源
        if self.cap.isOpened():
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    image_publisher_node = AprilTagDetectNode()
    rclpy.spin(image_publisher_node)
    image_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()