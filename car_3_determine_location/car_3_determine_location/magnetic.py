# !/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: 小车底盘数据发布
"""

import rclpy
import time
from rclpy.node import Node
from car_interfaces.msg import MagneticInterface, ComInterface

"""
@topic: magnetic_data pub
        float64  timestamp  #时间戳
        bool[] magneticlist #32位长度的磁条霍尔传感器数据
        float32  process_time     # 进程处理时间

@topic: com_data sub
        float64  timestamp  #时间戳
        string[]  serialportdata #串口上发数据
        float32  process_time    # 进程处理时间
"""


class PublisherNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        #参数服务
        self.declare_parameter("magnetic_run",False)
        self.magnetic_pub = self.create_publisher(
            MagneticInterface, "magnetic_data", 10)
        self.sub = self.create_subscription(
            ComInterface, 'com_data', self.listener_callback_com_data, 10)
        self._ = {'0': [0, 0, 0, 0],
                  '1': [0, 0, 0, 1],
                  '2': [0, 0, 1, 0],
                  '3': [0, 0, 1, 1],
                  '4': [0, 1, 0, 0],
                  '5': [0, 1, 0, 1],
                  '6': [0, 1, 1, 0],
                  '7': [0, 1, 1, 1],
                  '8': [1, 0, 0, 0],
                  '9': [1, 0, 0, 1],
                  'A': [1, 0, 1, 0],
                  'B': [1, 0, 1, 1],
                  'C': [1, 1, 0, 0],
                  'D': [1, 1, 0, 1],
                  'E': [1, 1, 1, 0],
                  'F': [1, 1, 1, 1]}

    def listener_callback_com_data(self,data_msg):
        magnetic_run = self.get_parameter('magnetic_run').get_parameter_value().bool_value
        # print("magnetic_run",magnetic_run)
        data = data_msg.serialportdata
        result = []
        if data[0] == '<':#and data[9] == 62
            for i in range(8):
                result += (self._[data[i + 1]])
            self.magnetic_msg_pub(result)
        else:
            # self.get_logger().info("data err >>")
            pass
    def magnetic_msg_pub(self, msg):
        magnetic_msg = MagneticInterface()
        re = []
        for i in msg:
            re.append(bool(i))
        magnetic_msg.magneticlist = re
        self.magnetic_pub.publish(magnetic_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("magnetic")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
