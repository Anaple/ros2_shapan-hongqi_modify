"""
@作者: kmakise
@说明: 车道线识别发布节点
"""

from car_config.config import CARTUEN_DEAFULT
import cv2
import numpy as np
import rclpy
from car_1_camera.LineDetect import LineDetect
from car_interfaces.msg import *
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

"""
@topic: camera_ori_data sub
        float32                 timestamp       # 时间戳
        uint8                   id              # 摄像头ID
        sensor_msgs/msg/Image   imagedata       # 相机图像
        float32                 process_time    # 进程处理时间

@topic: lane_recognition_data pub
        float32                 timestamp       #时间戳
        uint8                   id              # 摄像头ID
        float32                 centeroffset    # 车道中心线偏移距离
        sensor_msgs/Image       resultimage     # 融合后的车道线识别图像
        float32                 process_time    # 进程处理时间
"""


class PublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)
        # 车道线识别节点
        self.node = LineDetect()
        self.bridge = CvBridge()
        # 每隔5秒重新计算阈值参数
        self.interval = 5
        # 图像数据副本
        self.image = None
        # 当前车速 固定值,不考虑使用
        # self.fs = 0.0
        # 障碍物距离
        self.range = 9999
        # 上一次车道线中心值
        self.last_view_center = 240
        self.last_range = 0
        # 监听摄像头数据
        self.img_sub = self.create_subscription(Image, 'camera_ori_data', self.img_sub_callback, 10)
        # 监听超声数据
        self.sensor_sub = self.create_subscription(
            SonicObstacleInterface, 'sonic_obstacle_data', self.sensor_sub_callback, 10)
        # 发布控制数据
        self.control_pub = self.create_publisher(PidInterface, "pid_data", 10)
        # 发布图像处理数据
        self.img_pub = self.create_publisher(Image, "lane_recognition_Image_data", 10)
        # 启动一个定时器，定时重新计算阈值参数
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def img_sub_callback(self, data):
        # 将图片格式转换为opencv使用的格式
        frame = self.bridge.imgmsg_to_cv2(data)
        # 将图像旋转180°，安装的时候是倒着装的
        flip_img = cv2.flip(frame, -1)
        # 保存图像副本，供色彩平衡阈值计算使用
        self.image = flip_img
        # 获取色彩平衡后的图像
        color_balance = self.node.prev_cb_image(flip_img)
        # 查找白色边缘(轮廓)
        edge_white = self.node.get_edges(color_balance)
        # 查找白色线段
        white_lines = self.node.get_lines(edge_white)
        # 绘制车道线
        result_img = self.node.draw_lines(flip_img, white_lines)

        # 计算和上一次的偏差，不可以一次转动大角度
        difference = self.node.view_center - self.last_view_center
        if difference > 30:
            center = self.last_view_center + 30
        elif difference < -30:
            center = self.last_view_center - 30
        else:
            center = self.node.view_center

        # 限制(中心)方向最大值
        if center < 0:
            center = 0
        elif center > 480:
            center = 480

        # 保存上一次的车辆中心
        self.last_view_center = center
        # 计算车辆控制角度
        angle = center - 240
        angle = angle * 0.15

        _velocity = 90.0
        if self.range < 10:
            _velocity = 0.0
        elif self.range < 30:
            _velocity = 75.0

        ros_img = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
        self.img_pub.publish(ros_img)
        control = PidInterface()
        control.gear = 1
        control.angle = float(CARTUEN_DEAFULT - angle)
        control.velocity = _velocity
        control.throttle_percentage = 100
        control.braking_percentage = 0
        self.control_pub.publish(control)

    def sensor_sub_callback(self, data):
        self.range = data.ranges * 100
        print(self.range)

    def timer_callback(self):
        # 定时计算色彩平衡阈值
        self.node.calculate_new_parameters(self.image)


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("lane_recognition")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

