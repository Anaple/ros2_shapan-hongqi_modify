#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: 超声波雷达检测障碍物数据发布
"""

import rclpy  
import Jetson.GPIO as GPIO
import time

import transforms3d
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node                     
from car_interfaces.msg import SonicObstacleInterface
from tf2_ros import StaticTransformBroadcaster

"""
@topic: sonic_obstacle_data
        float32         timestamp       # 时间戳
        uint8           id              # 超声波雷达ID
        uint16          number          # 障碍物数量
        float32[]       obstacledata    # 障碍物数据
        float32         process_time    # 进程处理时间
        float32         angle           # 测量角度°,精度0.01
        float32         ranges          # 测量的距离数据[米],精度0.01

        其中obstacledata格式float32[]为一维数组,储存按照[angle, ranges,angle, ranges,……],2个长度数据为一组
"""


# Pin Definitions
trig_output_pin = 15  #发射PIN，J41_BOARD_PIN13---gpio14/GPIO.B06/SPI2_SCK
echo_input_pin = 16  #接收PIN，J41_BOARD_PIN18---gpio15/GPIO.B07/SPI2_CS0

class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                           
        self.get_logger().info("节点已启动：%s!" % name)
        self.pub = self.create_publisher(
            SonicObstacleInterface , "sonic_obstacle_data", 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.static_tf = StaticTransformBroadcaster(self)

        static_tf_msg = TransformStamped()
        static_tf_msg.header.stamp = self.get_clock().now().to_msg()
        static_tf_msg.header.frame_id = "base_link"
        static_tf_msg.child_frame_id = "sonic_obstacle"
        static_tf_msg.transform.translation.x = 0.12
        static_tf_msg.transform.translation.y = 0.0
        static_tf_msg.transform.translation.z = 0.07
        qua = transforms3d.euler.euler2quat(0.0,0.0,0.0)
        static_tf_msg.transform.rotation.x = qua[0]
        static_tf_msg.transform.rotation.y = qua[1]
        static_tf_msg.transform.rotation.z = qua[2]
        static_tf_msg.transform.rotation.w = qua[3]
        self.static_tf.sendTransform(static_tf_msg)


        
    def timer_callback(self): 
        gsod = SonicObstacleInterface()
        gsod.angle = 0.0
        ranges = self.srf05_getdis()/100
        if ranges > 0:
            gsod.ranges = ranges
            if gsod.ranges < 0.15:
                gsod.number = 1
            else:
                gsod.number = 0
            self.pub.publish(gsod)
            #self.get_logger().info('gsod_ar: "%f,%f"' % (gsod.angle, gsod.ranges))
        else:
            print("计算错误，检查超声链接线序")

    def srf05_getdis(self):
        GPIO.setmode(GPIO.BOARD)
        # set pin as an output pin with optional initial state of LOW
        GPIO.setup(trig_output_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(echo_input_pin, GPIO.IN)
        # Toggle the output every second
        GPIO.output(trig_output_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig_output_pin, GPIO.LOW)
        time_out = time.time()
        pulse_start = time.time()
        while GPIO.input(echo_input_pin) == 0:
            pulse_start = time.time()
            if time_out+0.1 < pulse_start:
                return -1

        time_out = time.time()
        pulse_end = time.time()
        while GPIO.input(echo_input_pin)==1:
            pulse_end = time.time()
            if time_out+0.1 < pulse_end:
                return -1

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        print ("Distance", distance)
        GPIO.cleanup()
        return distance


    
        
def main(args=None):                                
    rclpy.init(args=args)                           
    node = PublisherNode("sonic_obstacle")  
    rclpy.spin(node)                               
    node.destroy_node()                            
    rclpy.shutdown()              
