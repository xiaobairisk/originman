#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import dashscope
from concurrent.futures import ThreadPoolExecutor
import subprocess
import os
import logging
import re
import tempfile
import io

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')
        
        # 配置参数
        self.dashscope_api_key = "sk-3d037ba3824c40aba70b2593523ea4d0"
        dashscope.api_key = self.dashscope_api_key
        self.tts_model = "cosyvoice-v1"
        self.voice = "longcheng"
        self.audio_device = "plughw:1,0"
        
        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            String,
            'tts_input',
            self.text_callback,
            10
        )
        self.state_publisher = self.create_publisher(String, 'speech_state', 10)
        
        self.tts_executor = ThreadPoolExecutor(max_workers=4)
        
        # 启动时播放欢迎消息(单独使用时可以取消注释)
        # welcome_text = "你好，我是OriginMan，一个帅气的智能人形机器人。"
        # self.get_logger().info("节点启动，播放欢迎消息")
        # self._synthesize_and_play_stream(welcome_text)

        self.get_logger().info('使用示例：ros2 topic pub /tts_input std_msgs/msg/String "data: \'请告诉我今天的天气。\'"')
        self.get_logger().info("文本转语音节点已启动，等待输入/tts_input话题数据...")

    def text_callback(self, msg):
        """接收文本消息并处理"""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f"接收到文本: '{text}'")
            state_msg = String()
            state_msg.data = "speaking"
            self.state_publisher.publish(state_msg)
            self.get_logger().info("开始播报语音...")
            self._synthesize_and_play_stream(text)
            state_msg.data = "listening"
            self.state_publisher.publish(state_msg)
            self.get_logger().info("播报结束，进入聆听状态...")
        else:
            self.get_logger().info("接收到空文本，跳过处理")

    def _synthesize_segment(self, text):
        """合成单个句子并返回音频数据"""
        synthesizer = dashscope.audio.tts_v2.SpeechSynthesizer(model=self.tts_model, voice=self.voice)
        try:
            self.get_logger().info(f"开始合成文本: '{text}'")
            audio_data = synthesizer.call(text)
            if isinstance(audio_data, bytes):
                return audio_data
            else:
                self.get_logger().error(f"语音合成未生成有效数据: {text}, 返回值: {audio_data}")
                return None
        except Exception as e:
            self.get_logger().error(f"语音合成失败: {e}")
            return None

    def _convert_to_wav(self, mp3_data):
        """将 MP3 转换为 WAV 格式，匹配 AudioTestNode 的参数"""
        try:
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as mp3_file:
                mp3_file.write(mp3_data)
                mp3_path = mp3_file.name
            
            wav_path = tempfile.mktemp(suffix='.wav')
            cmd = [
                "ffmpeg",
                "-i", mp3_path,
                "-ac", "2",
                "-ar", "48000",
                "-f", "wav",
                "-acodec", "pcm_s32le",
                "-y",
                wav_path
            ]
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            
            with open(wav_path, 'rb') as f:
                wav_data = f.read()
            
            os.remove(mp3_path)
            os.remove(wav_path)
            return wav_data
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"MP3 转换 WAV 失败: {e.stderr.strip()}")
            return None
        except Exception as e:
            self.get_logger().error(f"转换过程中出错: {e}")
            return None

    def _play_audio(self, wav_data):
        """使用 aplay 播放 WAV 音频"""
        if wav_data:
            try:
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as wav_file:
                    wav_file.write(wav_data)
                    wav_path = wav_file.name
                
                cmd = ["aplay", "-D", self.audio_device, wav_path]
                result = subprocess.run(cmd, check=True, capture_output=True, text=True)
                
                os.remove(wav_path)
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"播放音频出错: {e.stderr.strip()}")
        else:
            self.get_logger().warning("音频数据为空，跳过播放")

    def _synthesize_and_play_stream(self, text):
        """将文本分段合成并流式播放"""
        if not text.strip():
            self.get_logger().info("无文本需要合成")
            return

        sentences = re.split(r'[.!?。！？]+', text)
        sentences = [s.strip() + '。' for s in sentences if s.strip()]
        self.get_logger().info(f"文本分割为 {len(sentences)} 个句子")

        futures = [self.tts_executor.submit(self._synthesize_segment, sentence) for sentence in sentences]
        for idx, future in enumerate(futures):
            mp3_data = future.result()
            if mp3_data:
                wav_data = self._convert_to_wav(mp3_data)
                if wav_data:
                    self.get_logger().info(f"播放第 {idx + 1} 段音频")
                    self._play_audio(wav_data)

        self.get_logger().info("所有音频段播放完成")

    def destroy_node(self):
        """节点关闭时的清理"""
        self.get_logger().info("正在关闭文本转语音节点")
        self.tts_executor.shutdown(wait=True)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，关闭节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()