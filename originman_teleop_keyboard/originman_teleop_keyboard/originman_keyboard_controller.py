#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, select, termios, tty
import time

import hiwonder.ActionGroupControl

class OriginmanKeyboardController(Node):
    def __init__(self):
        super().__init__('originman_keyboard_controller')
        self.publisher = self.create_publisher(String, 'robot_action_command', 10)
        self.get_logger().info('Originman Keyboard Controller started')

        # 启动时保持站立状态
        self.initial_stand()

    def initial_stand(self):
        """机器人启动时保持站立状态"""
        self.run_action('stand.dfa', 3.0)
        self.get_logger().info('机器人初始化为站立状态')

    def run_action(self, action_name, duration=1.0):
        """运行指定的动作并发布指令"""

        # 根据动作名称生成匹配的中文日志描述
        action_descriptions = {
            'go_forward.dfa': '前进',
            'back.dfa': '后退',
            'turn_left.dfa': '左转',
            'turn_right.dfa': '右转',
            '0.dfa': '双臂伸展',
            'bow.dfa': '鞠躬',
            'chest.dfa': '开怀大笑',
            'sit_ups.dfa': '仰卧起坐',
            'stand.dfa': '站立',
            'wave.dfa': '挥手',
            'wing_chun.dfa': '搓手',
        }
        description = action_descriptions.get(action_name, action_name)
        self.get_logger().info(f'发布动作指令：{description}，持续时间 {duration} 秒')

        msg = String()
        msg.data = description
        self.publisher.publish(msg)

        try:
            hiwonder.ActionGroupControl.runActionGroup(action_name.replace('.dfa', ''))
            time.sleep(duration)  # 等待动作执行完成
        except Exception as e:
            self.get_logger().error(f"执行动作 {action_name} 时出错: {str(e)}")

    def get_key(self):
        """读取键盘输入"""
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        """主循环，处理键盘输入"""
        msg = """
从键盘读取输入并发布到 robot_action_command!
---------------------------
控制机器人运动：
   w    i    o
   a    s    d
   q    x    c

u : 鞠躬
j : 仰卧起坐
k : 站立

CTRL-C 退出
"""
        print(msg)

        while True:
            key = self.get_key()
            if key == '\x03':  # CTRL-C
                self.get_logger().info('正在关闭...')
                break

            action_mappings = {
                'w': 'go_forward.dfa',  # 前进
                's': 'back.dfa',        # 后退
                'a': 'turn_left.dfa',   # 左转
                'd': 'turn_right.dfa',  # 右转
                'i': '0.dfa',           # 双臂伸展
                'o': 'chest.dfa',       # 开怀大笑
                'q': 'wave.dfa',        # 挥手
                'x': 'wing_chun.dfa',   # 搓手
                'c': 'sit_ups.dfa',     # 仰卧起坐
                'u': 'bow.dfa',         # 鞠躬
                'j': 'sit_ups.dfa',     # 仰卧起坐
                'k': 'stand.dfa',       # 站立
            }

            if key in action_mappings:
                self.run_action(action_mappings[key], 1.0)
            else:
                self.get_logger().info('无效按键。使用 w/a/s/d/i/o/q/x/c/u/j/k 或 CTRL-C 退出。')

def main(args=None):
    rclpy.init(args=args)
    controller = OriginmanKeyboardController()
    
    try:
        controller.run()
    except Exception as e:
        controller.get_logger().error(f"错误: {str(e)}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()