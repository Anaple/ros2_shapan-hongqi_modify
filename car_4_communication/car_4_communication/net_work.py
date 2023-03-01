#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: 网络节点
"""
import threading
from car_setting.car_setting import SOCKET_IP, SOCKET_PORT

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
import json
import time
import socket
import struct
from car_interfaces.msg import NetStationInterface, FusionInterface, GlobalPathPlanningInterface, CarDecisionInterface, \
    PidParameterInterface, NetLightInterface, NetEtcInterface, NetEtcControlInterface
from rclpy.publisher import Publisher
from std_msgs.msg import Int8
from std_msgs.msg import String

"""
@topic: net_light_data pub
        float32     timestamp       # 时间戳
        int8        number          # 在线红绿灯数量
        float32[]   lightdata       # 红绿灯状态数据
        float32[]   lighttimedata       # 红绿灯状态数据
        float32     process_time    # 进程处理时间

        float  longitude            # 经度
        float  attitude             # 纬度
        float  color                # 红绿灯当前状态(01:红,02:黄,03:绿,04:关闭）
        float  time                 # 倒计 时时长
        其中lightdata格式float32[]为一维数组,储存按照[ longitude,attitude,color,time, longitude,attitude,color,time……],4个长度数据为一组

@topic: net_station_data  pub
        float32     timestamp       # 时间戳
        float32[]   startpoint      # 起点位置,先经后纬
        float32[]   endpoint        # 终点位置,先经后纬
        float32     process_time    # 进程处理时间
"""


class Socker_client():
    def __init__(self, pub_net_station: Publisher, pub_net_cardecis: Publisher, pub_net_traffic: Publisher,
                 pub_pid: Publisher, pub_net_car_control: Publisher, pub_net_etc_state: Publisher):
        # self.screen_servr_ip = "10.1.1.198"
        self.screen_servr_ip = SOCKET_IP
        self.screen_prot = SOCKET_PORT
        self.nav_end_dic = {1: 26.0, 2: 23.0, 3: 21.0, 4: 16.0, 5: 12.0, 6: 11.0, 7: 5.0, 8: 4.0}
        self.pub_net_station = pub_net_station
        self.pub_net_cardecis = pub_net_cardecis
        self.pub_net_traffic = pub_net_traffic
        self.pub_pid = pub_pid
        self.pub_net_car_control = pub_net_car_control
        self.pub_net_etc_state = pub_net_etc_state
        self.server_is_debug = None
        self.Socket_Server_init(self.screen_servr_ip, self.screen_prot)

    def Socket_Server_init(self, host='192.168.0.182', port=61000):
        self.socker_c = socket.socket()  # 创建 socket 对象
        self.socker_c.connect((host, port))
        print("socker 已连接")
        # self.server_is_debug=True
        data = self.socker_c.recv(4)
        print(f"连接服务器4data :{data}")
        try:
            if data[0] == 255 and data[1] == 170:
                data_len = struct.unpack(">h", data[2:4])[0]
                data = self.socker_c.recv(data_len)
                a = json.loads(data)
                print("socker qt软件 ==>> 已连接")
                self.server_is_debug = True
                threading_1 = threading.Thread(target=self.Func_Recv_debug, args=())
                threading_1.setDaemon(True)
                threading_1.start()
            else:
                print("socker 大屏软件 ==>> 已连接")
                data = self.socker_c.recv(1024)
                self.server_is_debug = False
                threading_2 = threading.Thread(target=self.Func_Recv_, args=())
                threading_2.setDaemon(True)
                threading_2.start()

        except Exception as e:
            print(f"判断服务端类型失败：{e}")

    def Func_Recv_debug(self):
        """
        接收车辆消息
        :return:
        """
        netLightInterface = NetLightInterface()
        try:
            while self.socker_c:
                data = self.socker_c.recv(1)
                if data[0] == 255:
                    data = self.socker_c.recv(1)
                    if data[0] == 170:
                        data = self.socker_c.recv(2)
                        # print(f"data {data}")
                        data_len = struct.unpack(">h", data)[0]
                        # print("data_len{}".format(data_len))
                        try:
                            data = self.socker_c.recv(data_len)
                            recv_data_len = len(data)
                            while recv_data_len < data_len:
                                time.sleep(0.01)
                                # print(f"recv_data_len {recv_data_len}")
                                data += self.socker_c.recv(data_len - recv_data_len)
                                recv_data_len = len(data)
                                # if
                            # print(f"len(data) {len(data)}")
                            msg_msg = json.loads(data)
                            # print(time.time(),msg_msg)
                            type = msg_msg['type']
                            # print("type{}", format(type))
                            if type == "light":
                                # 红绿灯
                                light_list = msg_msg['light']
                                lightdata_list = []
                                lighttimedata_list = []
                                for i in light_list.values():
                                    lightdata_list.append(float(i['status']))
                                    lighttimedata_list.append(float(i['time']))
                                netLightInterface.number = len(light_list)
                                netLightInterface.lightdata = lightdata_list
                                netLightInterface.lighttimedata = lighttimedata_list
                                self.pub_net_traffic.publish(netLightInterface)

                            elif type == "mode_control":

                                mode_control = msg_msg['mode_control']
                                positioning = mode_control['positioning']  # 0巡磁/1动捕
                                Obstacle = mode_control['Obstacle']  # [0换道/1局部避障]
                                print("type==>mode_control  positioning{} Obstacle{}".format(positioning, Obstacle))
                                cardecisionInterface = CarDecisionInterface()
                                cardecisionInterface.driving_state = positioning
                                cardecisionInterface.come_across_obstacle_state = Obstacle
                                self.pub_net_cardecis.publish(cardecisionInterface)



                            elif type == "nav_end":
                                nav_end = msg_msg['nav_end']
                                plan_node = nav_end['plan_node']
                                print(f"type=>nav_end plan_node {plan_node}")
                                netStationInterface = NetStationInterface()
                                netStationInterface.startpoint = [0.0]
                                netStationInterface.endpoint = [float(plan_node)]
                                self.pub_net_station.publish(netStationInterface)

                            elif type == "etc_control":
                                etc_controls = msg_msg['etc_control']
                                # print(f"etc_controls =  {etc_controls}")
                                netetcinterface_msg = NetEtcInterface()
                                etc_list = []
                                num = 0
                                for etc_control in etc_controls.values():
                                    status = etc_control['status']
                                    etc_list.append(False if status else True)
                                    num += 1
                                netetcinterface_msg.status_list = etc_list
                                netetcinterface_msg.number = num
                                # print(f"netetcinterface_msg {netetcinterface_msg}")
                                self.pub_net_etc_state.publish(netetcinterface_msg)

                            elif type == "PID":
                                PID = msg_msg['PID']
                                P = PID['P']
                                I = PID['I']
                                D = PID['D']
                                pidParameterInterface = PidParameterInterface()
                                print(f"type=>PID {P} {I} {D}")
                                pidParameterInterface.p = float(P)
                                pidParameterInterface.i = float(I)
                                pidParameterInterface.d = float(D)
                                self.pub_pid.publish(pidParameterInterface)

                            elif type == "node_control":
                                node_control = msg_msg["node_control"]
                                camera = node_control['camera']
                                line = node_control['line']
                                print(f"type=>node_control camera {camera} line{line}")

                            elif type == "stop":
                                stop_msg = Int8()
                                stop_msg.data = 0
                                print(f"type=>stop")
                                self.pub_net_car_control.publish(stop_msg)

                            elif type == "start":
                                start_msg = Int8()
                                start_msg.data = 1
                                print(f"type=>start")
                                self.pub_net_car_control.publish(start_msg)
                    

                        except Exception as e:
                            print(f"err data {data}")



                else:
                    # print("type err", data)
                    pass
        except Exception as e:
            print("断开连接 {}".format(e))

    def Func_Recv_(self):
        """
        接收车辆消息
        :return:
        """
        try:
            while self.socker_c:
                data = self.socker_c.recv(1024)
                if data[0] == 255 and data[1] == 238 and data[5] == 254 and data[6] == 254:
                    msg_type = data[2]
                    # 沙盘消息广播
                    if msg_type == 12:
                        # msg_len = data[4]
                        hlv_len = 4
                        etc_len = 4
                        msg_len = struct.unpack(">h", data[3:5])[0]
                        msg = data[7:7 + msg_len]
                        if msg[0] == 255 and msg[1] == 51 and msg[msg_len - 2] == 243 and msg[msg_len - 1] == 243:
                            hlv_msg = {1: "红", 2: "绿", 3: "黄", 0: "关"}
                            rtc_msg = {0: "关", 1: "开"}
                            netLightInterface = NetLightInterface()
                            netLightInterface.number = 4
                            netLightInterface.lightdata = [float(i) for i in list(msg[2:2 + hlv_len])]
                            self.pub_net_traffic.publish(netLightInterface)

                    # 终点指令
                    elif msg_type == 8:
                        end_node = data[32]
                        print("终点指令", end_node)
                        netStationInterface = NetStationInterface()
                        netStationInterface.startpoint = [0.0]
                        netStationInterface.endpoint = [self.nav_end_dic[end_node]]
                        self.pub_net_station.publish(netStationInterface)


                    # 运动状态
                    elif msg_type == 1:
                        code = data[3]
                        yun_msg = {0: "急停", 1: "恢复运动"}
                        start_msg = Int8()
                        start_msg.data = code
                        print("运动状态", yun_msg[code])
                        self.pub_net_car_control.publish(start_msg)

        except Exception as e:
            print("err {}".format(e))

    def num_to_3_byte(self, data) -> bytes:
        """
        将-100000~100000 转换位3byte数据
        :param data:
        :return:
        """
        a1 = (((data << 8) & 0xFF000000) >> 24)
        a2 = (((data << 8) & 0x00FF0000) >> 16)
        a3 = (((data << 8) & 0x0000FF00) >> 8)
        msg = bytes()
        msg += struct.pack("B", a1)
        msg += struct.pack("B", a2)
        msg += struct.pack("B", a3)
        return msg

    def send_msg(self,msg):
        """
        发送消息，发送前做判断
        :param msg:
        :return:
        """
        try:
            self.socker_c.sendall(msg)
            # print(f"send_msg no err")
        except Exception as e:
            self.open_server()
            print(f"send_msg err {e}")
            self.socker_c.sendall(msg)



    def open_server(self):
        self.Socket_Server_init(self.screen_servr_ip, self.screen_prot)
        print(f"open_serve ")

    def send_fake_people_control(self,fake_people_id,status_msg):
        if self.server_is_debug:
            """
            {"type":"fake_people",
                "fake_people":
                {"id":xx,
                "status":[0/1]
             }
            """
            # print(f"send_fake_people_control start")
            json_msg = {"type": "fake_people","fake_people":{"id":fake_people_id,"status":status_msg}}
            # print(f"json_msg {json_msg}")
            msg = json.dumps(json_msg).encode()
            msg_len = len(msg)
            msg_h = bytes.fromhex("FF AA") + struct.pack(">h", msg_len) + msg
            self.send_msg(msg_h)

        else:
            print("服务端未连接")


    def send_global_Path_Planning(self, code, result_list):

        """
        发送车辆全局路径规划
        :param result_list:路径规划列表
        :return:
        """

        if self.server_is_debug == False:
            msg = bytes.fromhex("FF 11 02")
            result_len = len(result_list) * 2 + 4
            a = struct.pack(">h", result_len)
            msg += a
            msg += bytes.fromhex("F1 F1 FF 22")
            for i in result_list:
                msg += struct.pack(">h", i)
            msg += bytes.fromhex("F2 F2")
            self.send_msg(msg)

        elif self.server_is_debug == True:
            """
            {"type":"nav_plan_result",
            "code":[0/-1]
            "nav_plan_result":【1,2,3,4】,}
            """
            json_msg = {"type": "nav_plan_result", "code": 0 if code else -1, "nav_plan_result": result_list}
            msg = json.dumps(json_msg).encode()
            msg_len = len(msg)
            # print("msg_len {}".format(msg_len))
            msg_h = bytes.fromhex("FF AA") + struct.pack(">h", msg_len) + msg
            self.send_msg(msg_h)
        else:
            print("服务端未连接")

    def send_car_state(self, car_id=0, speed=0, angle=0, x=0, y=0, z=0, ax=0,
                       ay=0, az=0, stop=0, model=0, P=0, D=0, roundabout=0, tracing=0,
                       via_point=0, alleged_head1=0, alleged_head2=0, imu=0, obstacles=0,
                       to_stop_line=0, such_red_lights=0, traffic_light_status=0,
                       traffic_light_num=0, chassisonline=0, in_front_etc=0, such_etc=0,
                       etc_status=0, in_is_parking_space=0, such_parking_space=0, intparking_space_is_available=0,
                       location=0, biggest_angle=0, manual=0, tof=0, ultrasonic=0, ultrasonic_mun=0, check=0,
                       electricity=95):
        """
        发送车辆状态
        :param car_id:
        :param speed:
        :param angle:
        :param x:
        :param y:
        :param z:
        :param ax:
        :param ay:
        :param az:
        :param stop:
        :param model:
        :param P:
        :param D:
        :param roundabout:
        :param tracing:
        :param via_point:
        :param alleged_head1:
        :param alleged_head2:
        :param imu:
        :param obstacles:
        :param to_stop_line:
        :param such_red_lights:
        :param traffic_light_status:
        :param traffic_light_num:
        :param chassisonline:
        :param in_front_etc:
        :param such_etc:
        :param etc_status:
        :param in_is_parking_space:
        :param such_parking_space:
        :param intparking_space_is_available:
        :param location:
        :param biggest_angle:
        :param manual:
        :param tof:
        :param ultrasonic:
        :param ultrasonic_mun:
        :param check:Int8
        :return:
         """

        if self.server_is_debug == False:
            msg = bytes.fromhex("FF 11 01")
            result_len = 60
            msg += struct.pack(">h", result_len)
            msg += bytes.fromhex("F1 F1")
            msg += struct.pack("b", car_id)
            msg += struct.pack("b", speed)

            msg += self.num_to_3_byte(angle)
            msg += self.num_to_3_byte(x)
            msg += self.num_to_3_byte(y)
            msg += self.num_to_3_byte(z)
            msg += self.num_to_3_byte(ax)
            msg += self.num_to_3_byte(ay)
            msg += self.num_to_3_byte(az)

            msg += struct.pack("b", stop)
            msg += struct.pack("b", model)
            msg += struct.pack(">h", P)
            msg += struct.pack(">h", D)
            msg += struct.pack("b", roundabout)
            msg += struct.pack("b", tracing)
            msg += struct.pack("b", via_point)
            msg += struct.pack("b", alleged_head1)
            msg += struct.pack("b", alleged_head2)
            msg += struct.pack("b", imu)
            msg += struct.pack(">h", obstacles)
            msg += struct.pack("b", to_stop_line)
            msg += struct.pack("b", such_red_lights)
            msg += struct.pack("b", traffic_light_status)
            msg += struct.pack("b", traffic_light_num)
            msg += struct.pack("b", chassisonline)
            msg += struct.pack("b", in_front_etc)
            msg += struct.pack("b", such_etc)
            msg += struct.pack("b", etc_status)
            msg += struct.pack("b", in_is_parking_space)
            msg += struct.pack("b", such_parking_space)
            msg += struct.pack("b", intparking_space_is_available)
            msg += struct.pack("b", location)
            msg += struct.pack("b", biggest_angle)
            msg += struct.pack("b", manual)
            msg += struct.pack("b", tof)
            msg += struct.pack("b", ultrasonic)
            msg += struct.pack("b", ultrasonic_mun)
            msg += struct.pack("b", check)
            msg += bytes.fromhex("FA")
            self.send_msg(msg)
        elif self.server_is_debug == True:
            """
            {type:dev_status,
                {dev_status:
                    {speed:1,
                    angle:2,
                    camera:[0/1],
                    millimeter:[0/1],
                    ultrasonic:[0/1],
                    magnetic:[0/1],
                    verb:[0/1],
                    }
                }
            }
            """
            json_msg = {"type": "dev_status",
                        "dev_status": {"speed": speed, "angle": angle, "camera": 1, "millimeter": 1, "ultrasonic": 1,
                                       "magnetic": 1, "verb": 1, "electricity": electricity}}
            msg = json.dumps(json_msg).encode()
            msg_len = len(msg)
            # print("dev_status  json_msg{}".format(json_msg))
            msg_h = bytes.fromhex("FF AA") + struct.pack(">h", msg_len) + msg
            self.send_msg(msg_h)

    def send_etc_control(self, id_, status):
        """
        发送etc控制消息
        :param etc_msg:
        :return:
        {type:etc_control,
            etc_control:{id:xx,status:[0/1]
            }
        }
        """
        if self.server_is_debug == True:
            """
            {type:etc_control,
                {etc_control:
                    {id:xx,
                    status:[0/1]
                }

            }   
            """
            json_msg = {"type": "etc_control", "etc_control": {"id": id_, "status": 1 if status else 0}}
            msg = json.dumps(json_msg).encode()
            msg_len = len(msg)
            # print("etc_control  json_msg{}".format(json_msg))
            msg_h = bytes.fromhex("FF AA") + struct.pack(">h", msg_len) + msg
            self.send_msg(msg_h)

        elif self.server_is_debug == False:
            msg = bytes.fromhex("FF 11 04")
            msg += struct.pack(">h", id_)
            msg += bytes.fromhex("F1 F1")
            self.send_msg(msg)
        else:
            print("服务端未连接")


    def send_err_msg(self, err_msg):
        """
        发送错误消息
        :param err_msg:
        :return:msg_type
        """
        msg = bytes.fromhex("FF 11 03")
        msg += struct.pack(">h", err_msg)
        msg += bytes.fromhex("F1 F1")
        self.send_msg(msg)


class PublisherNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.pub_net_station = self.create_publisher(NetStationInterface, "net_station_data", 10)
        # 状态机
        self.pub_net_cardecis = self.create_publisher(CarDecisionInterface, "net_cardecis_data", 10)
        # 红绿灯
        self.pub_net_traffic = self.create_publisher(NetLightInterface, "net_traffic_sign_data", 10)
        # 发布小车控制
        self.pub_net_car_control = self.create_publisher(Int8, "net_car_control", 10)
        # 发布ect状态
        self.pub_net_etc_state = self.create_publisher(NetEtcInterface, "net_etc_state", 10)
        # pid
        self.pub_pid = self.create_publisher(PidParameterInterface, "net_pid_data", 10)

        #sub
        self.sub_fake = self.create_subscription(String, 'fake_people_control', self.listener_callback_fake_people_control, 1)
        self.sub_fus = self.create_subscription(FusionInterface, 'fusion_data', self.listener_callback_fsd, 1)
        self.sub_car_err_msg = self.create_subscription(Int8, 'car_err_data', self.listener_callback_car_err, 10)
        self.sub_global_plan_msg = self.create_subscription(GlobalPathPlanningInterface, 'global_path_planning_data',
                                                            self.listener_callback_global_plan, 10)
        self.sub_etc_control_msg = self.create_subscription(NetEtcControlInterface, 'etc_control',
                                                            self.listener_callback_etc_control, 10)
        self.socker_client = Socker_client(self.pub_net_station, self.pub_net_cardecis, self.pub_net_traffic,
                                           self.pub_pid, self.pub_net_car_control, self.pub_net_etc_state)



    def listener_callback_etc_control(self, data):
        id_ = data.id
        status = data.status
        self.socker_client.send_etc_control(id_, status)

    def listener_callback_car_err(self, data):
        err_msg = data.data
        if err_msg > 0:
            self.socker_client.send_err_msg(err_msg)
        elif err_msg == -1:
            self.socker_client.send_err_msg(200)

    def listener_callback_fake_people_control(self,data):
        data_msg = data.data
        data_msg_list = data_msg.split(",")
        fake_people_id = data_msg_list[0]
        status_msg = data_msg_list[1]
        # print(f"fake_people_id:{fake_people_id},status_msg:{status_msg}")
        self.socker_client.send_fake_people_control(fake_people_id,status_msg)

    def listener_callback_global_plan(self, data):
        routedata = data.routedata
        startpoint = data.startpoint
        endpoint = data.endpoint
        code = data.code
        """
        routedata = array('f', 
        [18.0, 1.9850000143051147, 4.396999835968018, 99.0, 50.0, 1.0, 3.0, 
        19.0, 2.194999933242798, 3.3949999809265137, 99.0, 50.0, 18.0, 3.0, 
        23.0, 0.8199999928474426, 3.1589999198913574, 99.0, 50.0, 19.0, 3.0]) 
        endpoint = array('f', [23.0, 0.8199999928474426, 3.1589999198913574]) 
        startpoint = array('f', [1.0, 0.47999998927116394, 4.091000080108643])
        """
        tmp = []
        if code:
            for i in self.list_of_groups(routedata, 7):
                tmp.append(int(i[0]))
            result = [int(startpoint[0])] + tmp
        else:
            result = []
            code = -1
        self.socker_client.send_global_Path_Planning(code, result)

    def list_of_groups(self, init_list, children_list_len):
        """
        收到的ros msg消息 以协议形式切割为children_list_len 位一组的列表
        :param init_list: 以为长度的列表
        :param children_list_len: 切片长度
        :return: 输出二维数组
        """
        list_of_groups = zip(*(iter(init_list),) * children_list_len)
        end_list = [list(i) for i in list_of_groups]
        count = len(init_list) % children_list_len
        end_list.append(init_list[-count:]) if count != 0 else end_list
        return end_list

    def pub_Navigation_msg(self, end_node):
        netstation_pub = NetStationInterface()
        netstation_pub.timestamp = time.time()
        netstation_pub.startpoint = [0.0]
        netstation_pub.endpoint = [float(end_node)]
        self.pub_net_station.publish(netstation_pub)

    def listener_callback_fsd(self, data):
        # self.get_logger().info('sub_t fsd : "%f"' % data.timestamp)
        car_speed = int(data.car_speed)
        Angle = int(data.yaw)
        longitude = int(data.longitude)
        latitude = int(data.latitude)
        height = int(data.height)
        ax = int(data.ax)
        ay = int(data.ay)
        az = int(data.az)
        gear_pos = int(data.gear_pos)
        soc = data.soc
        model = 0
        D = 0
        P = 0
        self.socker_client. \
            send_car_state(speed=car_speed, angle=Angle, x=longitude,
                           y=latitude, z=height, ax=ax, ay=ay, az=az,
                           stop=gear_pos, electricity=soc)


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("net_station")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()   
