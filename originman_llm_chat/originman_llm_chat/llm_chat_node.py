#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import os
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class Config:
    def __init__(self):
        self.llm_api_key = os.getenv('LLM_API_KEY', "fastgpt-xZUMZaHNVXbrDW8UiPpgfmN9BBFchTwQiZghRWnUHdZSbsT1OPp3if0lgl9")
        self.openai_base_url = "http://59.110.158.57:3000/api/v1"
        self.openai_model = "deepseek r1"
        self.max_history = 20

class LLMInfer:
    def __init__(self, config):
        self.ans_hist = []
        self.client = OpenAI(base_url=config.openai_base_url, api_key=config.llm_api_key)
        self.config = config

    def llm_infer(self, text):
        if not text:
            logging.info("用户输入为空，未调用语言模型")
            return ""
        messages = [{"role": "system", "content": "你是OriginMan。"}]
        for entry in self.ans_hist:
            messages.append({"role": "user", "content": entry["user"]})
            messages.append({"role": "assistant", "content": entry["assistant"]})
        messages.append({"role": "user", "content": [{"type": "text", "text": text}]})

        try:
            completion = self.client.chat.completions.create(
                model=self.config.openai_model,
                messages=messages,
                stream=True
            )
            response_message = ""
            for chunk in completion:
                if hasattr(chunk.choices[0].delta, 'content') and chunk.choices[0].delta.content is not None:
                    content = chunk.choices[0].delta.content
                    response_message += content
            self.ans_hist.append({"user": text, "assistant": response_message})
            if len(self.ans_hist) > self.config.max_history:
                self.ans_hist = self.ans_hist[-self.config.max_history:]
            logging.info(f"AI 响应: {response_message}")
            return response_message
        except Exception as e:
            logging.error(f"语言模型调用失败: {e}")
            return ""

class LLMChatNode(Node):
    def __init__(self):
        super().__init__('openai_interaction_node')
        self.config = Config()
        
        # 创建发布者，用于发送文本给 TTS 节点
        self.tts_publisher = self.create_publisher(String, 'tts_input', 10)
        
        # 创建订阅者，接收文本输入
        self.subscription = self.create_subscription(
            String,
            'text_input',
            self.text_callback,
            10
        )
        
        self.llm_inf = LLMInfer(self.config)
        
        self.get_logger().info("OpenAI 交互节点已启动，等待输入/text_input话题数据...")
        self.get_logger().info('使用示例：ros2 topic pub --once /text_input std_msgs/msg/String "data: \'你好，今天天气如何？\'"')

    def text_callback(self, msg):
        """接收文本消息并处理"""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f"接收到文本: '{text}'")
            response = self.llm_inf.llm_infer(text)
            if response:
                self._publish_response(response)
            else:
                self._publish_response("抱歉，我没听明白，请再说一遍")
        else:
            self.get_logger().info("接收到空文本，跳过处理")

    def _publish_response(self, text):
        """发布文本到 TTS 话题"""
        if text:
            msg = String()
            msg.data = text
            self.tts_publisher.publish(msg)
            self.get_logger().info(f"已发送 TTS 消息: '{text}'")

    def destroy_node(self):
        """节点关闭时的清理"""
        self.get_logger().info("正在关闭 OpenAI 交互节点")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LLMChatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，关闭节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()