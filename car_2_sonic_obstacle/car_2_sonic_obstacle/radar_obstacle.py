#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise 
@说明: milr雷达检测障碍物数据发布

@作者: noob 2020-11-30
@说明: milr雷达检测障碍物数据发布,更新毫米波协议
"""
import threading
from car_config.config import TTY_MILLIMETER

import rclpy
import time
import serial
from rclpy.node import Node
from car_interfaces.msg import RadarObstacleInterface
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d

"""
@topic: sonic_obstacle_data
        float32     timestamp          #时间戳
        uint8       id                   # 毫米波雷达ID
        uint16      number  #障碍物数量
        float32[]   obstacledata  #障碍物数据
        float32     process_time    # 进程处理时间
        其中obstacledata格式float32[]为一维数组,储存按照[category ,length ,width ,height , x  ,y  ,z  ,v  ,latv ,category  ,length ,width ,height , x ,y ,z ,v ,latv ,……]存储,9个长度数据为一组
        int         category     # 障碍物类别, 0点目标;1小汽车;2卡车/客车;3行人;4摩托车/电动车;5自行车;6宽大目标（如墙面）;7reserved保留; 8traffic light; 9stop sign
        float  length       # 障碍物长度m,精度为0.01
        float  width       # 障碍物宽度m,精度为0.01
        float  height       # 障碍物高度m,精度为0.01
        float  x     # 障碍物距离单位cm
        float  y   # 强度
        float  z    # 障碍物的z坐标[米],精度为0.01
        float  v=0    # 障碍物速度cm/s
        float  latv=0  # 障碍物的横向速度m/s,精度为0.01
        注 当障碍物类别存在而x,y,z值均为0,说明程序出现错误 主动式
"""


class PublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.pub = self.create_publisher(
            RadarObstacleInterface, "radar_obstacle_data", 10)
        self.serial = serial.Serial(TTY_MILLIMETER, 115200, timeout=0.5)
        # self.timer = self.create_timer(0.1, self.timer_callback)
        t = threading.Thread(target=self.receive_data, args=())
        t.start()
        self.msg = RadarObstacleInterface()
        self.create_tf()

    #创建tf
    def create_tf(self):
        self.static_tf = StaticTransformBroadcaster(self)
        static_tf_msg = TransformStamped()
        static_tf_msg.header.stamp = self.get_clock().now().to_msg()
        static_tf_msg.header.frame_id = "base_link"
        static_tf_msg.child_frame_id = "radar_obstacle"
        static_tf_msg.transform.translation.x = 0.12
        static_tf_msg.transform.translation.y = 0.0
        static_tf_msg.transform.translation.z = 0.05
        qua = transforms3d.euler.euler2quat(0.0, 0.0, 0.0)
        static_tf_msg.transform.rotation.x = qua[0]
        static_tf_msg.transform.rotation.y = qua[1]
        static_tf_msg.transform.rotation.z = qua[2]
        static_tf_msg.transform.rotation.w = qua[3]
        self.static_tf.sendTransform(static_tf_msg)

    # 子线程接收数据
    def receive_data(self):
        while True:
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                self.parse_protocol(data)

    # 解析协议
    def parse_protocol(self, data):
        #校验
        check_result = self.check_sum(data)
        # 前两位为帧头
        if data[0] == 0xFF and data[1] == 0x00 and check_result:
            # 第三位设备地址 #第四位帧长度
            device_address = data[2]
            frame_length = data[3]
            # 第五六位为距离数据
            distance_data = data[4] << 8 | data[5]
            # 第七八位为速度数据 有符号数
            u_speed_data = data[6] << 8 | data[7]
            speed_data = u_speed_data if u_speed_data < 32768 else u_speed_data - 65536
            # 第九位为信号强度
            signal_strength = data[8]
            # # 第十和十一位为校验位前面所有字节的累加和(按单字节加)
            # check_sum = data[9] << 8 | data[10]
            #self.get_logger().info(f"设备地址: {device_address},帧长度: {frame_length},距离数据(cm): {distance_data},速度数据(cm/s): {speed_data},信号强度: {signal_strength}")
            # 发布消息
            #速度单位为cm/s
            self.msg.v = speed_data
            #距离单位为cm
            self.msg.x = distance_data
            #信号强度
            self.msg.y = signal_strength
            self.pub.publish(self.msg)



    # 校验
    def check_sum(self, data):
        sum = 0
        # data前9位累加
        for i in range(9):
            sum += data[i]
        # 第十和十一位为校验位前面所有字节的累加和(按单字节加)
        check_sum = data[9] << 8 | data[10]
        if check_sum == sum:
            return True
        else:
            return False

    # 关闭串口
    def close_serial(self):
        self.ser.close()





def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("radar_obstacle")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
