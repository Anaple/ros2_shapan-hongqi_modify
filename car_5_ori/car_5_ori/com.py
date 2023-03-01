#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: 小车底盘控制
"""

import rclpy
from rclpy.node import Node
from car_interfaces.msg import PidInterface
from car_interfaces.msg import ComInterface
from car_setting.car_setting import TTY_THS1,CARTUEN_MSG
import serial
import serial.tools.list_ports
import threading


"""
@topic: pid_data sub
        float32     timestamp               # 数据帧时间戳
        float32     velocity                # 车辆速度m/s
        float32     angle                   # 车辆转角，方向盘转角 
        uint8       gear                    # 01:驻车:02:空挡(N):03:前进(D):04:后退(R):05:无效:
        uint8       throttle_percentage     # 油门踏板开度(0-100%)   取值:0-100
        uint8       braking_percentage      # 刹车踏板开度(0-100%) 取值: 0-100
        float32     process_time            # 进程处理时间
"""


def getAllCom():
    # 打印可用串口列表
    comlist = list(serial.tools.list_ports.comports())
    result = []
    if len(comlist) > 0:
        for i in comlist:
            result.append(str(i).split("-")[0])
    return result


class PublisherNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.old_speed = 0
        self.old_tuen = 50
        self.port = TTY_THS1
        self.baud = 115200
        self.serialportupload_pub = self.create_publisher(ComInterface, "/com_data", 10)
        self.sub = self.create_subscription(PidInterface, 'pid_data', self.listener_callback, 1)
        # self.timer = self.create_timer(1, self.timer_callback)
        self.open()
        threading.Thread(target=self.get_data, args=(1,)).start()

    def open(self):
        try:
            self.open_com = serial.Serial(self.port, self.baud, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                          stopbits=serial.STOPBITS_ONE)
            self.get_logger().info("com init ok >>")
        except Exception as e:
            print(e)

    def close(self):
        if self.open_com is not None and self.open_com.isOpen:
            self.open_com.close()
            self.get_logger().info("open_com.close ok")

    def pose_to_Pulse_number(self, dir, speet, tuen):
        str_sta = '<'
        str_end = '>'
        str_all = str_sta
        str_all += dir
        str_all += speet
        str_all += ',1'
        str_all += tuen
        str_all += str_end
        return str_all

    def send_str(self, sed_str):
        if self.open_com is None:
            self.open()
        else:
            sed_bytes = bytes(sed_str, encoding="gb18030")
            self.sed_bytes(sed_bytes)

    def Ascii_str(self, ascll):
        return chr(ascll)

    def sed_bytes(self, sed_ascii):
        success_bytes = self.open_com.write(sed_ascii)  # data.encode('UTF-8')

    def get_data(self, serial_port_callback):
        """
        多线程，回调形式处理收到的数据
        :param over_time:
        :return:
        """

        if self.open_com is None:
            self.open()
        while True:
            # data = self.data_filter(self.open_com.readline())
            data = self.open_com.readline()
            # print("data_1",data.decode("gb18030","ignore"),data[0],data[1],data[9],data[10])
            comInterface = ComInterface()
            str_dta = []
            for i in data:
                str_dta.append(str(chr(i)))
            comInterface.serialportdata = str_dta
            self.serialportupload_pub.publish(comInterface)




    def msg_callback(self, dir1, speet1, tuen1):
        """
        <140,100,190,100,100,100>
        电机1 电机2 舵机1 舵机2 舵机3 舵机4
        电机 1正反(1正转/2) 40速度(0~99)
        舵机 基础100 (0~180) 小车的中为190
        """
               
        speet1 = str(speet1) if 0<speet1 <=99 else "00"
        tuen1 = str(tuen1) if 0<tuen1 <=280 else "00"
        #logger.info("str speet1{}  tuen1  {}".format(speet1,tuen1))
        msg = CARTUEN_MSG(tuen1,dir1,speet1)
        self.get_logger().info("send msg  {}".format(msg))
        self.send_str(msg)
        self.old_speed = speet1
        self.old_tuen = tuen1



    def listener_callback(self, data):
        speed = int(data.velocity)
        angle = int(data.angle)
        gear = data.gear
        # gear 2  1 后 speed(0~100) angle(0~50~100)
        self.msg_callback(gear, speed, angle)


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("car_control")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
