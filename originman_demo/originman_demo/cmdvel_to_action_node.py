#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import hiwonder.ActionGroupControl
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class CmdVelToActionNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_action_node')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 动作组映射
        self.action_map = {
            'forward': 'go_forward',       # 前进
            'backward': 'back',            # 后退
            'turn_left': 'turn_left',      # 左转
            'turn_right': 'turn_right',    # 右转
            'stop': 'stand',               # 停止并站立
            'wave': 'wave'                 # 挥手（零速时）
        }
        
        # 速度阈值
        self.linear_threshold = 0.1    # m/s，线速度阈值
        self.angular_threshold = 0.1   # rad/s，角速度阈值
        
        # 上次执行的动作
        self.last_action = None
        
        # 启动时执行一次 stand 动作
        self.get_logger().info("CmdVel 转动作节点已启动，执行初始站立动作...")
        self.run_action_group(self.action_map['stop'])  # 执行 stand.dfa
        
        self.get_logger().info("CmdVel 转动作节点已就绪，等待 /cmd_vel 消息...")

    def cmd_vel_callback(self, msg):
        """处理 /cmd_vel 消息并转换为动作"""
        linear_x = msg.linear.x   # 前后速度
        angular_z = msg.angular.z # 左右旋转速度
        
        # 根据速度判断动作
        if abs(linear_x) < self.linear_threshold and abs(angular_z) < self.angular_threshold:
            action = 'wave'  # 零速时执行挥手
        elif linear_x > self.linear_threshold:
            action = 'forward'
        elif linear_x < -self.linear_threshold:
            action = 'backward'
        elif angular_z > self.angular_threshold:
            action = 'turn_left'
        elif angular_z < -self.angular_threshold:
            action = 'turn_right'
        else:
            action = 'stop'
        
        # 仅在动作变化时执行，避免重复
        if action != self.last_action:
            self.get_logger().info(f"收到速度命令: linear_x={linear_x}, angular_z={angular_z}, 执行动作: {action}")
            self.run_action_group(self.action_map[action])
            self.last_action = action

    def run_action_group(self, action_group_name, wait_time=5):
        """运行指定的动作组"""
        try:
            hiwonder.ActionGroupControl.runActionGroup(action_group_name)
            self.get_logger().info(f"执行动作组: {action_group_name}")
            time.sleep(wait_time)
        except Exception as e:
            self.get_logger().error(f"执行动作组 {action_group_name} 时出错: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，关闭节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()