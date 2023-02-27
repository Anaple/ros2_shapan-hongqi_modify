#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明:
"""
import math
import rclpy
import time
import transforms3d
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from car_interfaces.msg import GPSInterface, MagneticLocalPathPlanningInterface
from car_interfaces.msg import ImuInterface
from car_interfaces.msg import MagneticInterface
from car_interfaces.msg import NetLightInterface
from car_interfaces.msg import LaneRecognitionInterface
from car_interfaces.msg import SonicObstacleInterface
from car_interfaces.msg import CarOriInterface
from car_interfaces.msg import FusionInterface



"""
@topic: gps_data                sub
@topic: imu_data                sub
@topic: magnetic_data           sub
@topic: net_light_data          sub
@topic: lane_recognition_data   sub
@topic: sonic_obstacle_data     sub
@topic: car_ori_data            sub

@topic: fusion_data             pub

"""


class PublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.msg = FusionInterface()
        self.sub_gps = self.create_subscription(
            GPSInterface,
            'gps_data',
            self.listener_callback_gps, 1)
        self.sub_imu = self.create_subscription(
            ImuInterface,
            'imu_data',
            self.listener_callback_imu, 1)
        self.sub_magnetic = self.create_subscription(
            MagneticInterface,
            'magnetic_data',
            self.listener_callback_mgt, 1)
        self.sub_net_light = self.create_subscription(
            NetLightInterface,
            'net_light_data',
            self.listener_callback_nlt, 1)

        self.sub_car1 = self.create_subscription(
            PoseStamped,
            '/car2/pose',
            self.listener_callback_vrpn, 1)

        self.sub_lane = self.create_subscription(
            LaneRecognitionInterface,
            'lane_recognition_data',
            self.listener_callback_lnr, 1)

        self.sub = self.create_subscription(
            SonicObstacleInterface,
            'sonic_obstacle_data',
            self.listener_callback_sod, 1)
        self.sub_car_ori = self.create_subscription(
            CarOriInterface,
            'car_ori_data',
            self.listener_callback_ori, 1)

        self.magnetic_local_path_sub = self.create_subscription(
            MagneticLocalPathPlanningInterface,
            'magnetic_local_path_planning_data',
            self.magnetic_local_path_local, 1)

        self.pub_fusion = self.create_publisher(FusionInterface, "fusion_data", 1)
        # self.pub_pose = self.create_publisher(PoseStamped, "d_posr_data", 10)

        # self.timer = self.create_timer(0.05, self.timer_callback)

    # def timer_callback(self):
        #pass
        # self.get_logger().info('t:ssss "%d"' %  self.msg.timestamp )

    def listener_callback_gps(self, data):
        """
        uwb回调
        :param data:
        :return:
        """
        self.msg.longitude = data.longitude
        self.msg.latitude = data.latitude
        self.msg.timestamp = time.time()
        self.get_logger().info("{} {}".format(self.msg.longitude,self.msg.latitude))
        self.pub_fusion.publish(self.msg)


    def listener_callback_imu(self, data):
        """
        imu回调
        :param data:
        :return:
        """
        yaw = data.yaw
        roll = data.roll
        pitch = data.pitch
        yaw_angle = (yaw*180)/math.pi
        self.get_logger().info("yaw_angle {}".format(yaw_angle))
        self.msg.yaw = yaw_angle

    def listener_callback_mgt(self, data):
        # self.get_logger().info('sub_t: "%f"' % data.timestamp)
        pass

    def listener_callback_nlt(self, data):
        # self.get_logger().info('sub_t: "%f"' % data.timestamp)
        pass

    def listener_callback_lnr(self, data):
        # self.get_logger().info('sub_t: "%f"' % data.timestamp)
        pass
    def magnetic_local_path_local(self,data):
        car_stop = data.running_state
        if car_stop==1:
            self.msg.gear_pos = 1
        else:
            self.msg.gear_pos = 3
    def listener_callback_vrpn(self,data:PoseStamped):
        """
        动补回调
        :param data:
        :return:
        """
        self.msg.timestamp = time.time()
        self.msg.longitude = data.pose.position.x
        self.msg.latitude = data.pose.position.y
        self.msg.height = data.pose.position.z
        self.msg.ax = 0.0
        self.msg.ay = 0.0
        self.msg.az = 0.0
        angle = transforms3d.euler.quat2euler([data.pose.orientation.w,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z])
        yaw_angle = (angle[2]*180)/math.pi+180
        self.msg.yaw = yaw_angle
        # self.get_logger().info("{} {}".format(self.msg.longitude,self.msg.latitude))

    def listener_callback_sod(self, data):
        # self.get_logger().info('sub_t: "%f"' % data.timestamp)
        """
        超声回调
        :param data:
        :return:
        """
        try:
            #tf监听
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            from_frame_rel = "sonic_obstacle"
            to_frame_rel = "base_link"

            # now = self.get_clock().now().to_msg()
            # trans = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,now)  # 这里得到的是from_frame_rel在to_frame_rel坐标系下的位置
            """
            p2 = self.buffer.transform(p1, "base_link")
            只支持msgs_下的消息数据类型
            
            transforms3d库
            #从四元数创建旋转矩阵
            R = quat2mat([w, x, y, z])
            #创建一个旋转的矢量
            V = np.array([x, y, z]).reshape((3, 1))
            #旋转矢量
            M = np.dot(R, V)
            #旋转后数据
            p = PointStamped()
            p.point.x = M[0, 0]
            p.point.y = M[1, 0]
            p.point.z = M[2, 0]
            """
            #
            # t_x = trans.transform.translation.x
            # print("tf istener x {} ".format(t_x))
            # obstacle_dis = data.ranges-t_x
            obstacle_dis = data.ranges
            self.msg.obstaclenumber = data.number
            self.msg.obstacledata = [obstacle_dis]

            # print(trans.transform.translation.x)
            # print(trans.transform.rotation.x)

        except Exception as e:
            self.get_logger().error("超声节点异常 {}".format(e))

    def listener_callback_ori(self, data:CarOriInterface):
        """
        车辆底盘消息回调
        :param data:
        :return:
        """
        self.msg.car_speed = data.car_speed
        self.msg.soc = data.soc
        self.pub_fusion.publish(self.msg)





def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("fusion_data")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
