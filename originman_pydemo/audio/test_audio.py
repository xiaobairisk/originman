#!/usr/bin/env python3
import time
import subprocess
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from ros_robot_controller_sdk import Board
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class VoiceControlNode:
    def __init__(self):
        # 初始化Board
        self.board = Board()
        self.board.enable_reception()

        # 音频设备参数
        self.audio_device = "plughw:1,0"

    def play_audio(self, file_path):
        """播放音频文件"""
        try:
            cmd = ["aplay", "-D", self.audio_device, file_path]
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            logging.info(f"Played {file_path}")
        except subprocess.CalledProcessError as e:
            logging.error(f"Error playing {file_path}: {e.stderr.strip()}")

    def record_audio(self, file_path, duration=2):
        """录制音频，默认为2秒"""
        try:
            cmd = [
                "arecord",
                "-D", self.audio_device,
                "-c", "2",
                "-r", "48000",
                "-f", "S32_LE",
                "-t", "wav",
                "-d", str(duration),
                file_path
            ]
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            logging.info(f"Recorded {file_path}")
        except subprocess.CalledProcessError as e:
            logging.error(f"Error recording {file_path}: {e.stderr.strip()}")

    def run(self):
        """主循环，直接播放提示音、录制2秒并回放"""
        while True:
            self.play_audio("prompt.wav")  # 播放提示音
            self.record_audio("test.wav", duration=2)  # 录制2秒
            self.play_audio("test.wav")  # 回放录制的音频
            time.sleep(0.5)  # 短暂休眠，避免过于频繁循环

def main():
    # 输出代码使用说明
    logging.info("代码使用说明：")
    logging.info("1. 确保你的系统中已经安装了 `aplay` 和 `arecord` 工具，用于音频播放和录制。")
    logging.info("2. 确保 `prompt.wav` 文件存在于当前工作目录中，该文件将作为提示音播放。")
    logging.info("3. 运行此脚本后，程序将进入一个无限循环，每次循环会执行以下操作：")
    logging.info("   - 播放 `prompt.wav` 提示音。")
    logging.info("   - 录制2秒音频并保存为 `test.wav` 文件。")
    logging.info("   - 回放刚刚录制的 `test.wav` 文件。")
    logging.info("   - 短暂休眠0.5秒后，开始下一次循环。")
    logging.info("4. 若要停止程序，可在终端中按下 `Ctrl + C`。")

    node = VoiceControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        logging.info("Shutting down voice control")

if __name__ == "__main__":
    main()
