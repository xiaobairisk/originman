#!/usr/bin/env python3
# encoding:utf-8
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import hiwonder.ActionGroupControl
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def levenshtein_distance(s1, s2):
    """计算两个字符串的 Levenshtein 距离"""
    if len(s1) < len(s2):
        return levenshtein_distance(s2, s1)
    if len(s2) == 0:
        return len(s1)
    previous_row = range(len(s2) + 1)
    for i, c1 in enumerate(s1):
        current_row = [i + 1]
        for j, c2 in enumerate(s2):
            insertions = previous_row[j + 1] + 1
            deletions = current_row[j] + 1
            substitutions = previous_row[j] + (c1 != c2)
            current_row.append(min(insertions, deletions, substitutions))
        previous_row = current_row
    return previous_row[-1]

class ActionControlNode(Node):
    def __init__(self):
        super().__init__('action_control_node')
        
        # 创建订阅者，从 /text_input 接收语音命令
        self.subscription = self.create_subscription(
            String,
            'text_input',
            self.text_callback,
            10
        )
        
        # 动作组映射
        self.action_map = {
            '前进': 'go_forward',
            '后退': 'back',
            '左转': 'turn_left',
            '右转': 'turn_right',
            '双臂伸展': '0',
            '鞠躬': 'bow',
            '开怀大笑': 'chest',
            '仰卧起坐': 'sit_ups',
            '站立': 'stand',
            '挥手': 'wave',
            '搓手': 'wing_chun'
        }
        
        # 启动时执行站立动作
        self.get_logger().info("动作控制节点已启动，执行初始站立动作...")
        self.run_action_group("stand")
        self.get_logger().info("动作控制节点已就绪，等待语音命令...")

    def text_callback(self, msg):
        """处理语音命令并执行动作"""
        command = msg.data.strip()
        self.get_logger().info(f"收到语音命令: '{command}'")
        
        # 只取第一个词
        first_command = command.split()[0] if command else ""
        if not first_command:
            self.get_logger().warning("命令为空，忽略...")
            return
        
        # 模糊匹配
        matched_action = None
        min_distance = float('inf')
        threshold = 2  # 编辑距离阈值，可调整
        
        for action_key in self.action_map.keys():
            distance = levenshtein_distance(first_command, action_key)
            if distance < min_distance and distance <= threshold:
                min_distance = distance
                matched_action = action_key
        
        if matched_action:
            action_group_name = self.action_map[matched_action]
            self.get_logger().info(f"模糊匹配成功: '{first_command}' -> '{matched_action}'")
            self.run_action_group(action_group_name)
        else:
            self.get_logger().warning(f"未找到匹配的动作: '{first_command}'")

    def run_action_group(self, action_group_name, wait_time=5):
        """运行指定的动作组"""
        try:
            hiwonder.ActionGroupControl.runActionGroup(action_group_name)
            self.get_logger().info(f"执行动作组: {action_group_name}")
            time.sleep(wait_time)  # 等待动作组完成
        except Exception as e:
            self.get_logger().error(f"执行动作组 {action_group_name} 时出错: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ActionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，关闭节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()