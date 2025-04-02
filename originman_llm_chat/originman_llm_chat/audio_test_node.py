#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import time
import os
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class AudioTestNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        # 音频设备参数
        self.audio_device = "plughw:1,0"

        # 手动指定 prompt.wav 文件的绝对路径
        self.prompt_file = "/userdata/dev_ws/src/originman/originman_llm_chat/originman_llm_chat/prompt.wav"

        if not os.path.exists(self.prompt_file):
            self.get_logger().error("prompt.wav 文件不存在，请确保文件位于指定路径下")
            raise FileNotFoundError("prompt.wav not found")

        # 定时器，用于循环执行音频操作，默认每 3 秒执行一次（可调整）
        self.timer = self.create_timer(3.0, self.audio_callback)

        self.get_logger().info("语音控制节点已启动")
        self.log_usage_instructions()

    def log_usage_instructions(self):
        """输出代码使用说明"""
        self.get_logger().info("代码使用说明：")
        self.get_logger().info("1. 确保系统中已安装 `aplay` 和 `arecord` 工具，用于音频播放和录制。")
        self.get_logger().info("2. 确保 `prompt.wav` 文件存在于指定路径，该文件将作为提示音播放。")
        self.get_logger().info("3. 节点运行后，将每隔 3 秒执行以下操作：")
        self.get_logger().info("   - 播放 `prompt.wav` 提示音。")
        self.get_logger().info("   - 录制 2 秒音频并保存为 `test.wav` 文件。")
        self.get_logger().info("   - 回放刚刚录制的 `test.wav` 文件。")
        self.get_logger().info("4. 若要停止节点，可按 `Ctrl + C`。")

    def play_audio(self, file_path):
        """播放音频文件"""
        try:
            cmd = ["aplay", "-D", self.audio_device, file_path]
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.get_logger().info(f"播放音频: {file_path}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"播放 {file_path} 出错: {e.stderr.strip()}")

    def record_audio(self, file_path, duration=2):
        """录制音频，默认录制 2 秒"""
        try:
            cmd = [
                "arecord",
                "-D", self.audio_device,
                "-c", "2",  # 双声道
                "-r", "48000",  # 采样率 48kHz
                "-f", "S32_LE",  # 32位有符号小端格式
                "-t", "wav",  # WAV 格式
                "-d", str(duration),  # 录制时长
                file_path
            ]
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.get_logger().info(f"录制音频: {file_path}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"录制 {file_path} 出错: {e.stderr.strip()}")

    def audio_callback(self):
        """定时器回调函数，执行播放、录制和回放"""
        self.play_audio(self.prompt_file)  # 播放提示音
        self.record_audio("test.wav", duration=2)  # 录制 2 秒
        self.play_audio("test.wav")  # 回放录制的音频

    def shutdown(self):
        """节点关闭时的清理"""
        self.get_logger().info("正在关闭语音控制节点")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，关闭节点")
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()