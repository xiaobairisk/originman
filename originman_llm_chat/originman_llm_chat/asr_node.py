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
import pyaudio
import webrtcvad
import wave

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class Config:
    def __init__(self):
        self.dashscope_api_key = "sk-b8989a63c1c243a988ec7668006e3f90"
        self.model = "paraformer-realtime-v2"
        self.audio_device = "hw:1,0"
        self.channels = 2
        self.input_sample_rate = 48000
        self.output_sample_rate = 16000
        self.format = pyaudio.paInt16
        self.chunk_size = 960
        self.vad_mode = 1
        self.input_audio_file = "test.wav"
        self.chunk_duration_ms = 2000
        self.silence_threshold = 75
        self.noise_reduction_factor = 0.5
        self.timeout = 6

class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        
        self.config = Config()
        dashscope.api_key = self.config.dashscope_api_key
        
        self.text_publisher = self.create_publisher(String, 'text_input', 10)
        self.state_publisher = self.create_publisher(String, 'speech_state', 10)
        self.state_subscription = self.create_subscription(
            String,
            'speech_state',
            self.state_callback,
            10
        )
        
        self.is_speaking = False
        self.stop_interaction = False
        
        self.audio = pyaudio.PyAudio()
        self.vad = webrtcvad.Vad(self.config.vad_mode)
        
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
        self.stop_interaction = True
        self.audio_thread.join()
        self.audio.terminate()
        super().destroy_node()
        self.get_logger().info("ASR 节点已关闭")

    def record_audio(self):
        """使用 pyaudio 录制音频并进行VAD"""
        device_index = None
        for i in range(self.audio.get_device_count()):
            dev_info = self.audio.get_device_info_by_index(i)
            if self.config.audio_device in dev_info.get("name", ""):
                device_index = i
                self.get_logger().info(f"找到设备: {dev_info['name']}, index={device_index}")
                break
        if device_index is None:
            self.get_logger().error("未找到指定设备，使用默认设备")
            device_index = self.audio.get_default_input_device_info()['index']

        stream = self.audio.open(
            format=self.config.format,
            channels=self.config.channels,
            rate=self.config.input_sample_rate,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=self.config.chunk_size
        )
        
        frames = []
        silence_count = 0
        is_speech_detected = False
        start_time = time.time()
        
        noise_frames = []
        for _ in range(50):
            data = stream.read(self.config.chunk_size, exception_on_overflow=False)
            noise_frames.append(np.frombuffer(data, dtype=np.int16))
        noise_profile = np.mean(np.concatenate(noise_frames), axis=0)

        self.get_logger().info("开始录音，等待语音输入...")
        while not self.stop_interaction:
            if self.is_speaking:
                self.get_logger().info("检测到播报状态，中断录音...")
                break
                
            data = stream.read(self.config.chunk_size, exception_on_overflow=False)
            audio_chunk = np.frombuffer(data, dtype=np.int16)
            denoised_chunk = audio_chunk - (noise_profile * self.config.noise_reduction_factor)
            denoised_data = denoised_chunk.astype(np.int16).tobytes()
            
            mono_chunk = np.mean(np.frombuffer(denoised_data, dtype=np.int16).reshape(-1, self.config.channels), axis=1).astype(np.int16)
            is_speech = self.vad.is_speech(mono_chunk.tobytes(), self.config.input_sample_rate)
            self.get_logger().debug(f"VAD 输出: is_speech={is_speech}, 音频能量={np.max(np.abs(mono_chunk))}")

            if is_speech:
                silence_count = 0
                if not is_speech_detected:
                    self.get_logger().info("检测到语音，开始记录...")
                    is_speech_detected = True
                frames.append(denoised_data)
            elif is_speech_detected:
                silence_count += 1
                frames.append(denoised_data)
                if silence_count > self.config.silence_threshold:
                    self.get_logger().info("检测到语音结束，停止录音...")
                    break
            
            if time.time() - start_time > self.config.timeout:
                break

        stream.stop_stream()
        stream.close()

        if frames and not self.is_speaking:
            temp_file = "temp_record.wav"
            wf = wave.open(temp_file, 'wb')
            wf.setnchannels(self.config.channels)
            wf.setsampwidth(self.audio.get_sample_size(self.config.format))
            wf.setframerate(self.config.input_sample_rate)
            wf.writeframes(b''.join(frames))
            wf.close()

            cmd = [
                "ffmpeg",
                "-i", temp_file,
                "-ac", str(self.config.channels),
                "-ar", str(self.config.input_sample_rate),
                "-f", "wav",
                "-y",
                self.config.input_audio_file
            ]
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"录制音频保存为: {self.config.input_audio_file}")
            else:
                self.get_logger().error(f"FFmpeg 转换失败: {result.stderr}")
            os.remove(temp_file)
        else:
            self.get_logger().info("未检测到有效语音输入或被播报中断")

    def denoise_and_convert(self):
        # [保持不变]
        try:
            audio = AudioSegment.from_file(self.config.input_audio_file, format="wav").set_frame_rate(self.config.output_sample_rate)
            audio.export("temp_raw.wav", format="wav")
            audio_data, sample_rate = sf.read("temp_raw.wav")
            self.get_logger().info(f"采样率: {sample_rate}, 时长: {len(audio) / 1000} 秒")

            if audio_data.ndim > 1:
                audio_data = np.mean(audio_data, axis=1)

            chunk_samples = int(sample_rate * (self.config.chunk_duration_ms / 1000))

            def denoise_chunk(chunk):
                return nr.reduce_noise(y=chunk, sr=sample_rate)

            with ThreadPoolExecutor(max_workers=1) as executor:
                chunks = [audio_data[i:i + chunk_samples] for i in range(0, len(audio_data), chunk_samples)]
                denoised_chunks = list(executor.map(denoise_chunk, [c for c in chunks if len(c) > 0]))
                denoised_data = np.concatenate(denoised_chunks)

            sf.write("temp_denoised.wav", denoised_data, sample_rate)
            converted_audio = AudioSegment.from_file("temp_denoised.wav").set_channels(1).set_frame_rate(self.config.output_sample_rate)
            converted_audio.export(self.config.input_audio_file, format="wav")

            for temp_file in ["temp_raw.wav", "temp_denoised.wav"]:
                if os.path.exists(temp_file):
                    os.remove(temp_file)

        except Exception as e:
            self.get_logger().error(f"降噪和转换出错: {e}")
            raise

    def recognize_speech(self):
        # [保持不变]
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
                self.get_logger().info("进入监听状态，准备录音...")
                try:
                    self.record_audio()
                    if os.path.exists(self.config.input_audio_file) and not self.is_speaking:
                        self.denoise_and_convert()
                        text = self.recognize_speech()
                        if text and rclpy.ok():
                            self._publish_text(text)
                except Exception as e:
                    self.get_logger().error(f"处理流程出错: {e}")
            else:
                # self.get_logger().info("正在播报，暂停录音和识别...")
                pass
            time.sleep(0.1)  # 缩短间隔，提高响应速度

    def _publish_text(self, text):
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