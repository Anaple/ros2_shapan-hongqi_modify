#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
from car_setting.car_setting import TTY_IMU
import rclpy
import time
import math
import sys
import platform
import threading
import serial.tools.list_ports
import transforms3d
#from sensor_msgs.msg import Imu
from car_interfaces.msg import ImuInterface
from sensor_msgs.msg import MagneticField
from std_msgs.msg import String
from sensor_msgs.msg import Imu
#发布tf
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

#from quaternions import Quaternion as Quaternion
from rclpy.node import Node

class PublisherNode(Node):

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.declare_parameter("port")
        self.declare_parameter("baud")
        self.tf_pub = TransformBroadcaster(self)
        # self.python_version = platform.python_version()[0]
        self.version = 0
        self.readreg = 0
        self.key = 0
        self.flag = 0
        self.iapflag = 0
        self.recordflag = 0
        self.buff = {}
        self.calibuff = list()
        self.recordbuff = list()
        self.angularVelocity = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.angle_degree = [0, 0, 0]
        self.mag_offset = [0, 0, 0]
        self.mag_range = [0, 0, 0]
        self.wt_imu = serial.Serial()
        # self.baudlist = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]
        add_thread = threading.Thread(target = self.run)
        add_thread.start()

    def run(self):
        self.imu_node = Node("imu")
        port = TTY_IMU
        baudrate = 9600
        self.imu_msg = ImuInterface()
        self.mag_msg = MagneticField()
        self.ros_imu_msg = Imu()
        try:
            self.wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=10)
            if self.wt_imu.isOpen():
                self.get_logger().info("串口打开成功...")
            else:
                self.wt_imu.open()
                self.get_logger().info("打开串口成功...")
        except Exception as e:
            print(e)
            self.get_logger().info("串口打开失败")
            exit(0)
        else:
            # self.AutoScanSensor()
            self.imu_pub = self.create_publisher(ImuInterface, "/imu_data",10)
            self.ros_imu_pub = self.create_publisher(Imu, "/ros_imu_data",10)
            # self.mag_pub = self.create_publisher(MagneticField,"wit/mag", 10)

            while True:
                try:
                    buff_count = self.wt_imu.inWaiting()
                    if buff_count > 0 and self.iapflag == 0:
                        #print(buff_count)
                        buff_data = self.wt_imu.read(buff_count)
                        #if self.recordflag:
                            #iself.recordbuff = self.recordbuff + buff_data
                        for i in range(0, buff_count):
                            self.handleSerialData(buff_data[i])
                except Exception as e:
                    print("exception:" + str(e))
                    print("imu 失去连接，接触不良，或断线")
                    exit(0)



    # 校验
    def checkSum(self,list_data, check_data):
        return sum(list_data) & 0xff == check_data

    # 16 进制转 ieee 浮点数
    def hex_to_short(self,raw_data):
        return list(struct.unpack("hhhh", bytearray(raw_data)))

    # 处理串口数据
    def handleSerialData(self,raw_data):
        angle_flag = False
        self.buff[self.key] = raw_data
        self.key += 1
        #len 11
        if self.buff[0] != 0x55:
            self.key = 0
            return -1
        if self.key < 11:  #根据数据长度位的判断, 来获取对应长度数据
            return -1
        else:
            data_buff = list(self.buff.values())  # 获取字典所有 value
            if self.buff[1] == 0x51:
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    self.acceleration = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                else:
                    print('0x51 校验失败')

            elif self.buff[1] == 0x52:
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    self.angularVelocity = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in
                                       range(0, 3)]

                else:
                    print('0x52 校验失败')

            elif self.buff[1] == 0x53:
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    temp = self.hex_to_short(data_buff[2:10])
                    self.angle_degree = [temp[i] / 32768.0 * 180 for i in range(0, 3)]
                    self.version = temp[3]
                    angle_flag = True
                    #print("0x53  {} {} {}".format(self.angle_degree[0],self.angle_degree[1],self.angle_degree[2]))
                else:
                    print('0x53 校验失败')

            elif self.buff[1] == 0x54:
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    self.magnetometer = self.hex_to_short(data_buff[2:10])
                    if self.flag:
                        self.calibuff.append(self.magnetometer[0:2])
                else:
                    print('0x54 校验失败')

            elif self.buff[1] == 0x5f:
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    readval = self.hex_to_short(data_buff[2:10])
                    if self.readreg == 0x0b:
                        self.mag_offset = readval
                    else:
                        self.mag_range = readval

                else:
                    print('0x5f 校验失败')

            else:
                self.buff = {}
                self.key = 0

            self.buff = {}
            self.key = 0
            if angle_flag:
                stamp = self.get_clock().now().to_msg()

                self.ros_imu_msg.header.stamp = stamp
                self.ros_imu_msg.header.frame_id = "map"

                self.mag_msg.header.stamp = stamp
                self.mag_msg.header.frame_id = "map"
                angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
                # qua = Quaternion.from_euler(angle_radian)
                qua = transforms3d.euler.euler2quat(angle_radian[0], angle_radian[1], angle_radian[2])
                # qua = transforms3d.euler.euler2quat(0, 0, 0)
                # print(qua)
                self.imu_msg.yaw = angle_radian[2]
                self.imu_msg.pitch = angle_radian[1]
                self.imu_msg.roll = angle_radian[0]

                self.imu_msg.wx = float(self.angularVelocity[0])
                self.imu_msg.wy = float(self.angularVelocity[1])
                self.imu_msg.wz = float(self.angularVelocity[2])

                self.imu_msg.ax = float(self.acceleration[0])
                self.imu_msg.ay = float(self.acceleration[1])
                self.imu_msg.az = float(self.acceleration[2])



                self.mag_msg.magnetic_field.x = float(self.magnetometer[0])
                self.mag_msg.magnetic_field.y = float(self.magnetometer[1])
                self.mag_msg.magnetic_field.z = float(self.magnetometer[2])                                                             

                self.imu_pub.publish(self.imu_msg)
                # self.mag_pub.publish(self.mag_msg)

                t = TransformStamped()

                t.header.stamp = self.imu_node.get_clock().now().to_msg()
                t.header.frame_id = "map"
                t.child_frame_id = 'imu'

                t.transform.translation.x = 0.08
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.08 
                
                # t.transform.rotation.w = float(self.X_bat.orientation[0])
                # t.transform.rotation.x = float(self.X_bat.orientation[1])
                # t.transform.rotation.y = float(self.X_bat.orientation[2])
                # t.transform.rotation.z = float(self.X_bat.orientation[3])
                t.transform.rotation.x = qua[2]
                t.transform.rotation.y = qua[1]
                t.transform.rotation.z = qua[0]
                t.transform.rotation.w = qua[3]
                self.tf_pub.sendTransform(t)

                self.ros_imu_msg.orientation.z = qua[0]
                self.ros_imu_msg.orientation.y = qua[1]
                self.ros_imu_msg.orientation.x = qua[2]
                self.ros_imu_msg.orientation.w = qua[3]

                self.ros_imu_msg.angular_velocity.x =  float(self.angularVelocity[0])
                self.ros_imu_msg.angular_velocity.y = float(self.angularVelocity[1])
                self.ros_imu_msg.angular_velocity.z = float(self.angularVelocity[2])

                self.ros_imu_msg.linear_acceleration.x = float(self.acceleration[0])
                self.ros_imu_msg.linear_acceleration.y = float(self.acceleration[1])
                self.ros_imu_msg.linear_acceleration.z = float(self.acceleration[2])
                self.ros_imu_pub.publish(self.ros_imu_msg)
            return 0





    def callback(self,data):
        # global readreg, flag, calibuff, wt_imu, iapflag, mag_offset, mag_range, version, recordflag, baudlist
        unlock_imu_cmd = '\xff\xaa\x69\x88\xb5'
        reset_magx_offset_cmd = '\xff\xaa\x0b\x00\x00'
        reset_magy_offset_cmd = '\xff\xaa\x0c\x00\x00'
        reset_magz_offset_cmd = '\xff\xaa\x0d\x00\x00'
        enter_mag_cali_cmd = '\xff\xaa\x01\x09\x00'
        exti_cali_cmd = '\xff\xaa\x01\x00\x00'
        save_param_cmd = '\xff\xaa\x00\x00\x00'
        read_mag_offset_cmd = '\xff\xaa\x27\x0b\x00'
        read_mag_range_cmd = '\xff\xaa\x27\x1c\x00'
        reboot_cmd = '\xff\xaa\x00\xff\x00'
        reset_mag_param_cmd = '\xff\xaa\x01\x07\x00'
        set_rsw_demo_cmd = '\xff\xaa\x02\x1f\x00'  # output time acc gyro angle mag

        if "mag" in data.data:
            self.wt_imu.write(unlock_imu_cmd)
            time.sleep(0.1)
            self.wt_imu.write(reset_magx_offset_cmd)
            time.sleep(0.1)
            self.wt_imu.write(reset_magy_offset_cmd)
            time.sleep(0.1)
            self.wt_imu.write(reset_magz_offset_cmd)
            time.sleep(0.1)
            self.wt_imu.write(reset_mag_param_cmd)
            time.sleep(0.1)
            self.wt_imu.write(enter_mag_cali_cmd)
            time.sleep(0.1)
            self.flag = 1
            self.calibuff = []
            self.mag_offset = [0, 0, 0]
            self.mag_range = [500, 500, 500]
        elif "exti" in data.data:
            self.flag = 0
            self.wt_imu.write(unlock_imu_cmd)
            time.sleep(0.1)
            self.wt_imu.write(exti_cali_cmd)
            time.sleep(0.1)
            self.wt_imu.write(save_param_cmd)
            time.sleep(1)
            self.readreg = 0x0b
            self.wt_imu.write(read_mag_offset_cmd)
            time.sleep(1)
            self.readreg = 0x1c
            self.wt_imu.write(read_mag_range_cmd)
            time.sleep(1)
            datalen = len(self.calibuff)
            r = list()
            if datalen > 0:
                for i in range(datalen):
                    tempx = ((self.calibuff[i][0] - self.mag_offset[0]) * 2 / float(self.mag_range[0]))
                    tempy = ((self.calibuff[i][1] - self.mag_offset[1]) * 2 / float(self.mag_range[1]))
                    temp = tempx * tempx + tempy * tempy - 1
                    r.append(abs(temp))
                sumval = sum(r)
                r_n = float(sumval) / datalen
                if r_n < 0.05:
                    print('magnetic field calibration results are very good')
                elif r_n < 0.1:
                    print('magnetic field calibration results are good')
                else:
                    print('magnetic field calibration results is bad, please try again')
        elif "version" in data.data:
            print('sensor version is {}'.format(self.version))
        elif "begin" in data.data:
            pass
            # record_thread = threading.Thread(target=self.recordThread)
            # record_thread.start()
        elif "stop" in data.data:
            self.recordflag = 0
        elif "rate" in data.data:
            ratelist = [0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 125, 200]
            try:
                val = data.data[4:]
                rate = float(val)
                for i in range(len(ratelist)):
                    if rate == ratelist[i]:
                        val = i + 1
                        cmd = bytearray(5)
                        cmd[0] = 0xff
                        cmd[1] = 0xaa
                        cmd[2] = 0x03
                        cmd[3] = val
                        cmd[4] = 0x00
                        self.wt_imu.write(unlock_imu_cmd)
                        time.sleep(0.1)
                        self.wt_imu.write(cmd)
            except Exception as e:
                print(e)
        elif "baud" in data.data:
            try:
                val = data.data[4:]
                baud = float(val)
                for i in range(len(self.baudlist)):
                    if baud == self.baudlist[i]:
                        val = i + 1
                        cmd = bytearray(5)
                        cmd[0] = 0xff
                        cmd[1] = 0xaa
                        cmd[2] = 0x04
                        cmd[3] = val
                        cmd[4] = 0x00
                        self.wt_imu.write(unlock_imu_cmd)
                        time.sleep(0.1)
                        self.wt_imu.write(cmd)
                        time.sleep(0.1)
                        self.wt_imu.baudrate = baud
            except Exception as e:
                print(e)
        elif "rsw" in data.data:
            self.wt_imu.write(unlock_imu_cmd)
            time.sleep(0.1)
            self.wt_imu.write(set_rsw_demo_cmd)
            time.sleep(0.1)

    def thread_job(self):
        rclpy.spin()

    def AutoScanSensor(self):
        # global wt_imu, baudlist
        try:
            for baud in self.baudlist:
                read_cmd = '\xff\xaa\x27\x00\x00'
                self.wt_imu.baudrate = baud
                self.wt_imu.flushInput()
                self.wt_imu.write(read_cmd)
                time.sleep(0.2)
                buff_count =self.wt_imu.inWaiting()
                if buff_count >= 11:
                    buff_data = self.wt_imu.read(buff_count)
                    val = bytearray(buff_data)
                    for i in range(len(val)):
                        if val[i] == 0x55:
                            sumval = sum(val[i:i + 10])
                            if sumval == val[i + 10]:
                                return

        except Exception as e:
            print("exception:" + str(e))
            print("imu 失去连接，接触不良，或断线")
            # exit(0)
def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("imu_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



