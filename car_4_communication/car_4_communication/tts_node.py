import os
import pyttsx3
import rclpy
from car_interfaces.msg import FusionInterface
from rclpy.node import Node
from std_msgs.msg import String


class TtsNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.engine = pyttsx3.init()
        # 获取语音包
        voices = self.engine.getProperty('voices')
        # print(f"voices {voices}")
        # for voice in voices:
        #     print(f'id = {voice.id}\tname = {voice.name} \n')
        self.engine.setProperty('voice', 'zh+f2') #开启支持中文
        # self.engine.setProperty('voice','english+f2')
        self.engine.setProperty('rate', 200)
        self.sub_tts_string = self.create_subscription(String,'tts_play_string',self.tts_play_string, 10)

    def tts_play_string(self,tts_string):
        text = tts_string.data
        print(f"tts text {text}")
        self.engine.say(text)
        self.engine.runAndWait()


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode("tts_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
