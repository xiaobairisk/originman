#!/usr/bin/env python3
# encoding:utf-8
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.bridge = CvBridge()
        # 创建发布者，发布 image_raw 话题
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        # 打开摄像头，0 表示默认摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
        else:
            # 创建定时器，定期发布图像
            self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                # 将 OpenCV 图像转换为 ROS 图像消息
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                # 发布图像消息
                self.publisher_.publish(ros_image)
                self.get_logger().info('发布图像')
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
    image_publisher_node = ImagePublisherNode()
    rclpy.spin(image_publisher_node)
    image_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()