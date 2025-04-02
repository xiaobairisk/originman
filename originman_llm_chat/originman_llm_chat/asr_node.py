#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import dashscope
from dashscope.audio.asr import Recognition
from http import HTTPStatus
import subprocess
import os
import threading
import time
import logging
from pydub import AudioSegment
import noisereduce as nr
import numpy as np
import soundfile as sf
from concurrent.futures import ThreadPoolExecutor

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class Config:
    def __init__(self):
        self.dashscope_api_key = "sk-3d037ba3824c40aba70b2593523ea4d0"
        self.model = "paraformer-realtime-v2"
        self.audio_device = "hw:1,0"  # 与 arecord 一致
        self.channels = 2             # 立体声
        self.input_sample_rate = 48000  # 输入采样率
        self.output_sample_rate = 16000  # Dashscope 支持的采样率
        self.format = "S32_LE"        # 32位有符号小端
        self.duration = 3            # 录音时长 10 秒
        self.input_audio_file = "test.wav"
        self.chunk_duration_ms = 2000  # 每段降噪 2 秒

class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        
        # 配置参数
        self.config = Config()
        dashscope.api_key = self.config.dashscope_api_key
        
        # 创建发布者
        self.text_publisher = self.create_publisher(String, 'text_input', 10)

        self.state_publisher = self.create_publisher(String, 'speech_state', 10)
        self.state_subscription = self.create_subscription(
            String,
            'speech_state',
            self.state_callback,
            10
        )
        # 状态标志
        self.is_speaking = False
        
        # 控制交互循环
        self.stop_interaction = False
        
        # 初始化线程池用于降噪（避免与 ROS 2 executor 重名）
        self.denoise_executor = ThreadPoolExecutor(max_workers=1)
        
        # 启动音频处理线程
        self.get_logger().info("ASR 节点已启动，开始录音和识别...")
        self.audio_thread = threading.Thread(target=self.audio_loop)
        self.audio_thread.start()

    def state_callback(self, msg):
        """接收语音状态"""
        state = msg.data
        if state == "speaking":
            self.is_speaking = True
            self.get_logger().info("收到播报状态，正在说话，暂停录音...")
        elif state == "listening":
            self.is_speaking = False
            self.get_logger().info("收到监听状态，开始聆听中...")

    def destroy_node(self):
        """节点关闭时的清理"""
        self.stop_interaction = True
        self.audio_thread.join()
        self.denoise_executor.shutdown(wait=True)
        super().destroy_node()
        self.get_logger().info("ASR 节点已关闭")

    def record_audio(self):
        """使用 arecord 录制音频"""
        try:
            cmd = [
                "arecord",
                "-D", self.config.audio_device,
                "-c", str(self.config.channels),
                "-r", str(self.config.input_sample_rate),
                "-f", self.config.format,
                "-t", "wav",
                "-d", str(self.config.duration),
                self.config.input_audio_file
            ]
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.get_logger().info(f"录制音频: {self.config.input_audio_file}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"录制音频出错: {e.stderr.strip()}")
            raise

    def denoise_and_convert(self):
        """降噪并转换音频格式"""
        try:
            # 加载音频并降低采样率
            audio = AudioSegment.from_file(self.config.input_audio_file, format="wav").set_frame_rate(self.config.output_sample_rate)
            audio.export("temp_raw.wav", format="wav")
            audio_data, sample_rate = sf.read("temp_raw.wav")
            self.get_logger().info(f"采样率: {sample_rate}, 时长: {len(audio) / 1000} 秒")

            # 如果是多声道，转为单声道
            if audio_data.ndim > 1:
                audio_data = np.mean(audio_data, axis=1)

            # 分段处理参数
            chunk_samples = int(sample_rate * (self.config.chunk_duration_ms / 1000))

            # 并行降噪
            def denoise_chunk(chunk):
                return nr.reduce_noise(y=chunk, sr=sample_rate)

            with ThreadPoolExecutor(max_workers=1) as executor:
                chunks = [audio_data[i:i + chunk_samples] for i in range(0, len(audio_data), chunk_samples)]
                denoised_chunks = list(executor.map(denoise_chunk, [c for c in chunks if len(c) > 0]))
                denoised_data = np.concatenate(denoised_chunks)

            # 保存降噪后的音频
            sf.write("temp_denoised.wav", denoised_data, sample_rate)
            converted_audio = AudioSegment.from_file("temp_denoised.wav").set_channels(1).set_frame_rate(self.config.output_sample_rate)
            converted_audio.export(self.config.input_audio_file, format="wav")

            # 删除临时文件
            for temp_file in ["temp_raw.wav", "temp_denoised.wav"]:
                if os.path.exists(temp_file):
                    os.remove(temp_file)

        except Exception as e:
            self.get_logger().error(f"降噪和转换出错: {e}")
            raise

    def recognize_speech(self):
        """使用 Dashscope 识别音频"""
        try:
            recognition = Recognition(
                model=self.config.model,
                format="wav",
                sample_rate=self.config.output_sample_rate,
                language_hints=['zh', 'en'],
                callback=None
            )
            result = recognition.call(self.config.input_audio_file)
            
            if result.status_code == HTTPStatus.OK:
                sentences = result.get_sentence()
                full_content = ""
                if sentences:
                    for sentence in sentences:
                        text = sentence.get('text', '')
                        if text:
                            full_content += text
                    if full_content:
                        self.get_logger().info(f"识别到用户语音: '{full_content}'")
                        return full_content
                    else:
                        self.get_logger().info("未识别到有效语音")
                        return ""
                else:
                    self.get_logger().info("未识别到任何内容")
                    return ""
            else:
                self.get_logger().error(f"识别失败: {result.message}")
                return ""
        except Exception as e:
            self.get_logger().error(f"语音识别出错: {e}")
            return ""

    def audio_loop(self):
            """音频处理循环"""
            while not self.stop_interaction:
                if not self.is_speaking:
                    state_msg = String()
                    state_msg.data = "listening"
                    self.state_publisher.publish(state_msg)
                    self.get_logger().info("开始录音和识别...")
                    try:
                        self.record_audio()
                        self.denoise_and_convert()
                        text = self.recognize_speech()
                        if text and rclpy.ok():
                            self._publish_text(text)
                    except Exception as e:
                        self.get_logger().error(f"处理流程出错: {e}")
                else:
                    self.get_logger().info("正在播报，暂停录音...")
                time.sleep(1)  # 每轮间隔 1 秒

    def _publish_text(self, text):
        """发布识别的文本到话题"""
        msg = String()
        msg.data = text
        self.text_publisher.publish(msg)
        self.get_logger().info(f"已发布文本到 /text_input: '{text}'")

def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，关闭节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()