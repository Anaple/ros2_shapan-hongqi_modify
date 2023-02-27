#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: 小车底盘数据发布
"""

import rclpy     
import time                              
from rclpy.node import Node                     
from car_interfaces.msg import CarOriInterface, ComInterface

"""
@topic: car_ori_data pub
        float32     timestamp               # 时间戳
        uint8       id                      # 车辆ID
        float32     car_speed               # 车辆速度,单位为m/s
        float32     steer_angle             # 车辆转角,左转为正,右转为负
        int8        gearPos                 # 车辆档位信号(01:驻车,02:空挡(N),03:前进(D),04:后退(R),05:无效)
        float32     brake_tq                # 制动量(-50-50nm)
        uint8       parking_state           # 制动状态(00:驻车状态,01:驻车释放状态)
        uint8       soc                     # 电池当前SOC(0-100)
        uint8       battery_vol             # 电池电压(0-55V)
        uint16      battery_discharge_cur   # 电池放电电流(0-100A)
        uint8       car_run_mode            # 车辆运行模式: 0: ACU控制  1:自动驾驶  2: 急停模式
        uint8       throttle_percentage     # 油门踏板开度: 取值0~100
        uint8       braking_percentage      # 制动踏板开度: 取值0~100
        bool        left_light              # 左转向灯状态:0:关闭,1:打开
        bool        right_light             # 右转向灯状态:0:关闭,1:打开
        bool        reversing_light         # 倒车灯状态:0:关闭,1:打开
        bool        speaker                 # 喇叭状态:0:关闭,1:打开
        bool        start_button            # 启动按钮状态:0:按键无效,1:按键有效
        bool        stop_button             # 急停按钮状态:0:按键无效,1:按键有效
        uint8       state                   # 设备状态,0:状态正常,1:电池箱报警:2:电机控制器报警
        uint8       error                   # 错误码:电池箱报警:1:单体过压或欠压,2:放电电流异常,3:电压报警,4:电池温度报警,5:电池SOC过低。电机控制器报警:1:转向电机控制器故障,2:驱动电机控制器故障
        float32     process_time            # 进程处理时间

"""

class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.min_voltage = 11.1
        self.max_voltage = 12.6
        self.soc = 0
        self.o_time = 0
        self.cnt_distance = 0.31/20
        self.cnt_num = 0
        self.distance = 0
        self.car_speed = 0
        self.pub = self.create_publisher(
            CarOriInterface , "car_ori_data", 10)
        self.sub = self.create_subscription(
            ComInterface, 'com_data', self.listener_callback_com_data, 10)

    def electricity_calculate(self,voltage):
        # self.get_logger().error("voltage {}".format(voltage))
        return int(((voltage-self.min_voltage)/(self.max_voltage-self.min_voltage))*100)
    def speed_calculate(self,time,distance):
        return distance/time
    def distance_calculate(self,cnt):
        """
        根据编码器计算运动距离
        :param cnt:编码器数值
        :return:距离
        """
        #编码器值 * 一个编码器数值代表的小车运动距离
        return cnt*self.cnt_distance

    def listener_callback_com_data(self,data):
        """
        底盘数据解析计算
        :param data:
        :return:
        """
        try:
            com_msg = data.serialportdata
            com_str ="".join(com_msg)
            if com_msg[0]=="<":
                com_msg_list = com_str.split(",")
                #得到电压数据
                voltage_str = com_msg_list[1]
                #电压换算
                voltage = float(voltage_str)/100
                #调用函数根据电压计算百分比电量
                self.soc = self.electricity_calculate(voltage)
                #获取编码器数值
                cnt = int(com_msg_list[2])
                #根据编码器计算运动距离
                distance = self.distance_calculate(cnt)
                #累加距离
                self.distance+=distance
                #累加编码器数量
                self.cnt_num+=cnt
                #记录时间
                n_time = time.time()
                if self.o_time!=0:
                    #计算速度 距离 / 时间
                    self.car_speed = self.speed_calculate(n_time,distance)
                self.o_time = n_time
                #发布数据
                pub = CarOriInterface()
                pub.timestamp = time.time()
                pub.car_speed = float(self.car_speed)
                # self.get_logger().error("self.soc ={}".format(self.soc))

                # pub.soc = self.soc
                #todo soc err
                pub.brake_tq = float(self.cnt_num)
                self.pub.publish(pub)
            else:
                self.get_logger().info('com_msg err ')

        except Exception as e:
            self.get_logger().error(e)


        
def main(args=None):                                
    rclpy.init(args=args)                           
    node = PublisherNode("car_ori")  
    rclpy.spin(node)                               
    node.destroy_node()                            
    rclpy.shutdown()  
