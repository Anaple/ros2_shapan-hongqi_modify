#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: 局部路径规划
"""
import copy
import threading
import time
import cv2
import numpy as np
import rclpy
from car_interfaces.msg import SonicObstacleInterface
from car_interfaces.msg import FusionInterface, CarDecisionInterface, NetLightInterface, NavigationalStateInterface, \
    PidParameterInterface, NetEtcInterface,NetEtcControlInterface
from car_interfaces.msg import MagneticInterface, MagneticLocalPathPlanningInterface, GlobalPathPlanningInterface
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8

"""
@topic: global_path_planning_data sub

        float32     timestamp       # 时间戳
        float32[]   startpoint      # 起点位置，先经后纬
        float32[]   endpoint        # 终点位置，先经后纬
        float32[]   routedata       # 路径集合（所有途径点的集合）
        float32     process_time    # 进程处理时间
        float32     longitude       # 途径点经度
        float32     latitude        # 途径点纬度
        float32     speed           # 途径点速度
        float32     angle           # 途径点速度角度（航向角角度）

        其中routedata格式float32[]为一维数组，储存按照[longitude,latitude,
        speed,angle,longitude,latitude,speed, angle, ……]存储,4个长度数据为一组

@topic: fusion_data sub
        float32  timestamp  #时间戳
        uint8  id   # 车辆ID
        float32  car_length         # 车长m
        float32  car_width         # 车宽m
        float32  car_height         # 车高m--symlink-install
        float32  yaw         # 偏航角（0-359.99）
        float32  pitch        # 俯仰角（process_time=0.0-90-90）
        float32  roll         # 横滚角（-180-180）
        float32  wx          # 角速度x
        float32  wy          # 角速度y
        float32   wz         # 角速度z
        float32  ax          # 加速度x
        float32  ay          # 加速度y
        float32  az          # 加速度z
        float32   longitude        # 经度
        float32   latitude         # 纬度
        float32  height            # 高度
        float32   eastvelocity         # 东向速度
        float32   northvelocity        # 北向速度
        float32   skyvelocity         # 天向速度
        float32   car_speed    # 车辆速度（0-20km/h）
        float32   steer_angle    # 车辆转角（-28-28度）
        uint8  gear_pos    # 车辆档位信号（01：驻车，02：空挡，03：前进，04：后退，05：无效）
        float32  brake_tq    # 制动力矩（-50-50nm）
        uint8  parking_state    # 制动状态（00：驻车状态，01：驻车释放状态）
        uint8  soc    # 电池当前SOC（0-100）
        uint8  battery_vol    # 电池电压（0-55V）
        uint8  battery_discharge_cur    # 电池放电电流（0-100A）
        float32   camcenteroffset     # 目标识别车道中心线偏移量
        float32   magcenteroffset     # 磁导航磁线中心线偏移量
        uint16  obstaclenumber  #障碍物数量
        float32[]  obstacledata  #障碍物数据
        int8   lightnumber  #在线红绿灯数量
        float32[]  lightdata   # 红绿灯状态数据
        float32  process_time     # 进程处理时间


@topic: magnetic_data sub
        float64  timestamp  #时间戳
        bool[] magneticlist #32位长度的磁条霍尔传感器数据
        float32  process_time     # 进程处理时间


@topic: car_decision_data sub   
        float64  timestamp  #时间戳
        uint8  driving_state  #行驶方式 0动补定位行驶  1磁线行驶
        uint8  come_across_obstacle_state  #遇到障碍物处理方式 0停车等待  1换道行驶 2局部避障（回到原车道行驶）
        float32  process_time    # 进程处理时间

        
        
@topic: net_traffic_sign_data sub   
        float64  timestamp  #时间戳
        uint8  driving_state  #行驶方式 0动补定位行驶  1磁线行驶
        uint8  come_across_obstacle_state  #遇到障碍物处理方式 0停车等待  1换道行驶 2局部避障（回到原车道行驶）
        float32  process_time    # 进程处理时间   


@topic: magnetic_local_path_planning_data  pub
        float32     timestamp  #时间戳
        float32[]   startpoint   #起点位置，先经后纬
        float32[]   endpoint    #终点位置，先经后纬
        float32[]   routedata   #路径集合（所有途径点的集合）
        float32     process_time    # 进程处理时间

        float       longitude        # 途径点经度
        float       latitude         # 途径点纬度
        float       speed         # 途径点速度
        float       angle         # 途径点速度角度（航向角角度）
        其中routedata格式float32[]为一维数组，储存按照[longitude,latitude,
        speed,angle,longitude,latitude,speed, angle, ……]存储,4个长度数据为一组



"""



class Local_nav():
    def __init__(self, pub,pub_car_err:Publisher,pub_nav_data:Publisher,pub_img:Publisher,pub_etc_control_msg:Publisher):
        # car_pub 发布节点
        self.pub = pub
        self.pub_car_err = pub_car_err
        self.pub_nav_data = pub_nav_data
        self.pub_img = pub_img
        self.pub_etc_control_msg = pub_etc_control_msg
        self.image = Image()
        self.bridge = CvBridge()

        # global_path_msg  接受全局路径规划消息
        # 开始节点位置
        self.startpoint = None
        # 结束节点位置
        self.endpoint = None
        # 规划路径的节点
        self.routedata = None
        # 是否在执行导航
        self.nav_ing = False
        self.suspended = False
        # 车辆是否在运动
        self.car_run = False
        # 车辆的位置信息
        self.longitude = None
        self.latitude = None
        self.yaw_angle = None
        self.builtins_flag = False

        # 障碍物信息
        self.obstaclenumber = 0
        self.obstacle_dis = 99.99

        # stop_car_distance 遇到障碍物停车阈值
        self.stop_car_distance = 0.2
        # deviation 定位误差阈值
        self.deviation = 0.1

        # 将小车垂直地图log方向放置,查看偏航角
        self.Yaw_error = None

        # 虚拟点的数量
        self.expand_num = 5
        # magnetic
        self.magnetic_list = []
        self.magnetic_img_num = 30
        self.magnetic_flag = False
        self.magnetic_turn = 0
        self.magnetic_state = 1
        self.magnetic_result_list = []
        self.magnetic_centre_num = None
        # rfid 业务
        self.rfid_siteiId_dic = {26: 1, 23: 2, 21: 3, 16: 4, 12: 5, 11: 6, 5: 7, 4: 8}
        self.rfid_traffic_lights_dic = {15: 1, 14: 2, 20: 3, 19: 4}
        self.rfid_etc_dic = {3: 1, 6: 2,7:2, 24: 3, 25: 4}

        self.rfid_business_list = list(self.rfid_etc_dic) + list(self.rfid_siteiId_dic) + list(
            self.rfid_traffic_lights_dic)

        # self.traffic_lights_rfid =  {5:1,6:2,7:3,8:4}
        self.traffic_lights = {1:False,2:True,3:False}
        self.rfid = 0
        self.xx_id = 0

        self.navigationalStateInterface = NavigationalStateInterface()
        self.magneticLocalPathPlanningInterface_msg = MagneticLocalPathPlanningInterface()
        self.err_msg = Int8()
        self.come_across_obstacle_state = 1
    def stop_car(self):
        """
        发送停车消息 消息为ros下msg speed 为0停车 angle为90，舵机回正
        :return: 无
        """
        self.car_run = False
        pub_msg = MagneticLocalPathPlanningInterface()
        pub_msg.running_state = 1
        print("stop_car")
        self.pub.publish(pub_msg)


    def restore_car(self):
        """
        障碍物小时,车辆恢复运动
        :return: 无
        """
        self.car_run = True
        pub_msg = MagneticLocalPathPlanningInterface()
        pub_msg.running_state = 2
        #print("restore_car")
        self.pub.publish(pub_msg)

    def send_etc(self,etc_id,status):
        netEtcControlInterface = NetEtcControlInterface()
        netEtcControlInterface.id = etc_id
        netEtcControlInterface.status=status
        self.pub_etc_control_msg.publish(netEtcControlInterface)


    def movement_car(self, centeroffset):
        """
        发送车辆控制消息
        centeroffset
        :return: 无
        """
        self.car_run = True
        pub_msg = MagneticLocalPathPlanningInterface()
        pub_msg.running_state = 3
        pub_msg.centeroffset = centeroffset
        print("movement_car")
        self.pub.publish(pub_msg)

    def net_cardecis_msg_callback(self,driving_state, come_across_obstacle_state):
        """
        qt软件消息回调函数
        :param driving_state: 行驶方式 0动补定位行驶  1磁线行驶
        :param come_across_obstacle_state: #遇到障碍物处理方式 0停车等待  2局部避障
        :return:
        """
        self.driving_state = driving_state
        self.come_across_obstacle_state = come_across_obstacle_state

    def global_path_msg_callback(self, startpoint, endpoint, routedata):
        """
        全局路径规划消息回调函数
        :param startpoint: 收到的规划起点位置
        :param endpoint: 收到的规划终点位置
        :param routedata: 收到的规划所有节点位置及运动速度，角度
        :return: 无
        """
        # [0]start_node [1]start_node_x [1]start_node_y
        self.startpoint = startpoint
        # [0]end_node [1]end_node_x [1]end_node_y
        self.endpoint = endpoint
        # a = [::6] a[0]node a[1]node_x a[2]node_Y a[3]speed a[4]angle [5]magnetic_state
        self.routedata = routedata


    def list_of_groups(self, init_list, children_list_len):
        """
        收到的ros msg消息 以协议形式切割为children_list_len位一组的列表
        :param init_list: 以为长度的列表
        :param children_list_len: 切片长度
        :return: 输出二维数组
        """
        list_of_groups = zip(*(iter(init_list),) * children_list_len)
        end_list = [list(i) for i in list_of_groups]
        count = len(init_list) % children_list_len
        end_list.append(init_list[-count:]) if count != 0 else end_list
        return end_list




    def run(self):
        self.nav_ing = True
        self.car_run = True
        #self.come_across_obstacle_state = 1
        self.obstacle_flag = False
        nav_msg_list = self.list_of_groups(self.routedata, 7)
        print("nav_msg_list{} startint {} endpoint {}".format(nav_msg_list,self.startpoint[0], self.endpoint[0]))
        print("开始局部导航")
        self.navigationalStateInterface.start = int(self.startpoint[0])
        self.navigationalStateInterface.end = int(self.endpoint[0])
        self.navigationalStateInterface.save_flag = True
        self.pub_nav_state_data(self.navigationalStateInterface)
        while True:
            #检测是否到达终点
            if self.detection_end():
                # 策略更新
                self.strategy_updata(nav_msg_list)
                #交通判断
                if self.detection_traffic_light():
                    #检测障碍物
                    self.detection_obs()
                time.sleep(1/20)
            else:
                return 0

    def motion_control(self):
        """
        运动控制
        :return:
        """
        magnetic_r_list = copy.deepcopy(self.magnetic_result_list)
        ma_num = len(magnetic_r_list)
        #print(ma_num,magnetic_r_list)
        if ma_num > 0:
            if self.magnetic_state == 1:
                # 左
                # print("左")
                self.magnetic_turn = 16 - magnetic_r_list[0]


            elif self.magnetic_state == 2:
                # 中
                # print("中")
                if ma_num == 1:
                    self.magnetic_turn = 16 - magnetic_r_list[0]
                elif ma_num == 2:
                    self.magnetic_turn = 16 - magnetic_r_list[1]
                elif ma_num == 3:
                    self.magnetic_turn = 16 - magnetic_r_list[1]

            elif self.magnetic_state == 3:
                # 右
                # print("右")
                if  ma_num==1:
                    self.magnetic_turn=16 - magnetic_r_list[0]
                elif ma_num==2:
                    self.magnetic_turn = 16 - magnetic_r_list[1]
                elif ma_num==3:
                    self.magnetic_turn = 16 - magnetic_r_list[2]
            self.magneticLocalPathPlanningInterface_msg.running_state = 3
            self.magneticLocalPathPlanningInterface_msg.centeroffset = float(self.magnetic_turn)
            self.pub_magneticLocalPathPlann_data(self.magneticLocalPathPlanningInterface_msg)
            self.car_run = True
        else:
            self.magneticLocalPathPlanningInterface_msg.running_state = 1
            self.pub_magneticLocalPathPlann_data(self.magneticLocalPathPlanningInterface_msg)
            self.car_run = False

    def strategy_updata(self,nav_msg_list):
        """
        策略更新
        :param nav_msg_list:
        :return:
        """
        for i in nav_msg_list:
            if int(i[5]) == self.rfid:
                self.magnetic_state = int(i[6])

    def detection_traffic_light(self):
        """
        交通判断
        :return:
        """
        if self.rfid in self.rfid_business_list and self.builtins_flag:
            self.builtins_flag = False
            #etc
            if self.rfid in self.rfid_etc_dic:
                etc_id = self.rfid_etc_dic[self.rfid]
                self.stop_car()
                self.send_etc(etc_id,True)
                time.sleep(3)
                self.restore_car()

            elif self.rfid in self.rfid_traffic_lights_dic:
                # print("红绿灯")
                #判断运行状态，右转不需要看灯
                if self.magnetic_state<3:
                    self.hld = self.get_traffic_lights(self.rfid)
                    # print("红灯停车")
                    self.stop_car()
                    while self.get_traffic_lights(self.rfid):
                        time.sleep(0.1)
                    self.restore_car()
                    # print("绿灯恢复运动")
        else:
            return True

    def detection_obs(self):
        """
        障碍物判断
        :return:
        """

        #有障碍物
        if self.obstacle_dis < 0.3:
            #车在运行
            if self.car_run:
                print("==有障碍物==")
                # 自动避障
                if self.come_across_obstacle_state:
                    self.avoid_obstacle_stop()
                # 局部自动躲避障碍
                else:
                    self.avoid_obstacle_change_track()
            # 车在停止
            # else:
                #print("车辆停止状态")
        #没有障碍物
        else:
            if not self.car_run:
            #判断障碍物状态改变,发送车辆状态信息
                self.err_msg.data = 30
                self.pub_car_err_data(self.err_msg)
                self.restore_car()
            #运动控制
            self.motion_control()

    def avoid_obstacle_stop(self):
        """
        局部避障
        :return:
        """
        if self.obstaclenumber > 0:
            print("局部避障")
            self.stop_car()
            self.err_msg.data = 7
            self.pub_car_err_data(self.err_msg)

    def avoid_obstacle_change_track(self):
        """
        换道避障
        :return:
        """
        print("换道避障")
        time_sleep = 30
        print("右传")
        for i in range(time_sleep-3):
            self.magneticLocalPathPlanningInterface_msg.running_state = 3
            self.magneticLocalPathPlanningInterface_msg.centeroffset = float(-5)
            self.pub_magneticLocalPathPlann_data(self.magneticLocalPathPlanningInterface_msg)
            time.sleep(0.1)

        print("左转")
        for i in range(time_sleep*2+3):
            self.magneticLocalPathPlanningInterface_msg.running_state = 3
            self.magneticLocalPathPlanningInterface_msg.centeroffset = float(5)
            self.pub_magneticLocalPathPlann_data(self.magneticLocalPathPlanningInterface_msg)
            time.sleep(0.1)


    def detection_end(self):
        """
        检测是否到达终点
        :return:
        """
        if self.rfid != self.endpoint[0]:
            return True
        else:
            print("局部导航 结束")
            self.magneticLocalPathPlanningInterface_msg.running_state = 1
            self.pub_magneticLocalPathPlann_data(self.magneticLocalPathPlanningInterface_msg)
            self.err_msg.data = 3
            self.pub_car_err_data(self.err_msg)
            self.navigationalStateInterface.save_flag = False
            self.pub_nav_state_data(self.navigationalStateInterface)
            self.nav_ing = False
            self.car_run = False
            return False

    def pub_car_err_data(self,err_msg):
        """
        pub 车辆状态结果
        :param err_msg:
        :return:
        """
        self.pub_car_err.publish(err_msg)

    def pub_nav_state_data(self,navigationalStateInterface):
        """
        pub 导航状态即如果
        :return:
        """
        self.pub_nav_data.publish(navigationalStateInterface)

    def pub_magneticLocalPathPlann_data(self, magneticLocalPathPlanningInterface_msg):
        """
        pub 磁条处理结果
        :param magneticLocalPathPlanningInterface_msg:
        :return:
        """
        self.pub.publish(magneticLocalPathPlanningInterface_msg)
        self.magnetic_flag = False

    def get_traffic_lights(self,rfid):
        """
        传入rfid，返回红绿灯状态
        :param rfid:
        :return:
        """

        return self.traffic_lights[self.lightdata_int_list[self.rfid_traffic_lights_dic[rfid]]]


    def pub_img_data(self, data):
        self.img = np.ones((self.magnetic_img_num, 32, 1), dtype="uint8") * 255
        for i in range(len(data)):
            for j in range(len(data[0])):
                # self.img[i,j] = 0 if data[i][j] == True else 255
                if data[i][j] == True:
                    self.img[i, j] = 0
        img_test2 = cv2.resize(self.img, (0, 0), fx=10, fy=10, interpolation=cv2.INTER_NEAREST)
        self.image.data = self.bridge.cv2_to_imgmsg(img_test2, "")
        self.pub_img.publish(self.image)


    def magnetic_centre_and_magnetic_num(self, data):
        """
        传入传感数据，返回每条路的中心磁条id
        :param data:
        :return:返回检测到的路的中间磁条
        """
        ma_list = []
        tmp_list = []
        flag = False
        result_list = []
        for i in range(len(data)):
            if data[i] == True:
                # self.get_logger().error("i={}".format(i))
                tmp_list.append(i)
                me_num = len(tmp_list)
                flag = True
                if me_num > 25:
                    return []
            elif flag:
                ma_list.append(tmp_list)
                tmp_list = []
                flag = False

        for i in ma_list:
            me_index = int(len(i) / 2)
            # self.get_logger().error("me_index={} i={}".format(me_index, i))
            centre_index = i[me_index]
            result_list.append(centre_index)
        return result_list


    def rfid_msg_callback(self, rfid_data):
        self.rfid = rfid_data
        self.builtins_flag = True
        if rfid_data == 8:
            self.come_across_obstacle_state = 0 
        elif rfid_data == 9:
            self.come_across_obstacle_state = 1


    def magnetic_msg_callback(self, magnetic_data):
        """
        磁条消息回调
        :param magnetic_data:
        :return:
        """

        if len(self.magnetic_list) == self.magnetic_img_num:
            self.magnetic_list.pop(0)
        self.magnetic_list.append(magnetic_data)
        #if len(self.magnetic_list) == self.magnetic_img_num:
        #    self.pub_img_data(self.magnetic_list)
        self.magnetic_turn = 0
        self.magnetic_result_list = self.magnetic_centre_and_magnetic_num(magnetic_data)
        if len(self.magnetic_result_list)>0:
            self.magnetic_flag = True
        # self.get_logger().error("magnetic len {}".format(len(result_list)))


    def timer_callback(self):
        magneticLocalPathPlanningInterface_msg = MagneticLocalPathPlanningInterface()
        magneticLocalPathPlanningInterface_msg.centeroffset = float(self.magnetic_turn)
        if self.magnetic_flag:
            magneticLocalPathPlanningInterface_msg.running_state = 3
        else:
            magneticLocalPathPlanningInterface_msg.running_state = 1
        self.pub.publish(magneticLocalPathPlanningInterface_msg)
        self.magnetic_turn = 0
        self.magnetic_flag = False


    def fusion_msg_callback(self, longitude, latitude, obstaclenumber, obstacledatas, yaw_angle):
        """
        融合节点消息回调from std_msgs.msg import String
        :param longitude:x uwb小车位置x
        :param latitude:y uwb小车位置y
        :param obstaclenumber: 障碍物数量
        :param obstacledatas:rfid_msg_callback 障碍物数据
        :return:
        """
        self.longitude = longitude
        self.latitude = latitude
        #self.obstaclenumber = obstaclenumber

    def sod_msg_callback(self,obstacle_dis,obstaclenumber):
        self.obstaclenumber = obstaclenumber
        self.obstacle_dis = obstacle_dis
    def net_traffic_msg_callback(self,traffic_number:int,lightdata_int_list):
        self.traffic_number = traffic_number
        self.lightdata_int_list = lightdata_int_list


class PublisherNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub_magnetic_local_path = self.create_publisher(MagneticLocalPathPlanningInterface, "magnetic_local_path_planning_data", 1)
        self.pub_car_err = self.create_publisher(Int8, 'car_err_data', 1)
        self.pub_nav_data = self.create_publisher(NavigationalStateInterface, 'nav_data', 1)
        self.pub_img = self.create_publisher(Image, "magnetic_Image_data", 10)
        self.pub_etc_control_msg = self.create_publisher(NetEtcControlInterface, 'etc_control',1)


        self.local_nav = Local_nav(self.pub_magnetic_local_path,self.pub_car_err,self.pub_nav_data,self.pub_img,self.pub_etc_control_msg)
        self.sub_globalpath = self.create_subscription(GlobalPathPlanningInterface, 'global_path_planning_data',
                                            self.listener_callback_gpp, 10)

        self.sub_net_traffic = self.create_subscription(NetLightInterface, 'net_traffic_sign_data',
                                            self.listener_callback_net_traffic, 10)
        self.sub_net_cardecis = self.create_subscription(CarDecisionInterface, "net_cardecis_data",
                                                         self.listener_callback_net_cardecis,10)

        self.sub_sod = self.create_subscription(SonicObstacleInterface,'sonic_obstacle_data',self.listener_callback_sod, 1)
        self.fusion_sub = self.create_subscription(FusionInterface, 'fusion_data', self.listener_callback_fsd, 1)
        self.magnetic_sub = self.create_subscription(MagneticInterface, 'magnetic_data',
                                                     self.listener_callback_magnetic, 1)
        self.rfid_sub = self.create_subscription(String, "rfid_data", self.listener_callback_rfid, 1)
        # self.client_ = self.create_client(GlobalPathPlanningInterface, "Global_Path_Plan")
        self.get_logger().info("节点已启动：%s!" % name)



    def listener_callback_net_cardecis(self,data):
        """
        状态机状态回调
        :param data:
        :return:
        """
        driving_state  = data.driving_state
        come_across_obstacle_state = data.come_across_obstacle_state
        #self.local_nav.net_cardecis_msg_callback(driving_state, come_across_obstacle_state)

    def listener_callback_net_traffic(self,data):
        """
        沙盘广播回调
        :param data:
        :return:
        """
        traffic_number = data.number
        lightdata_list = data.lightdata
        lightdata_int_list = [int(i) for i in lightdata_list]
        self.local_nav.net_traffic_msg_callback(traffic_number,lightdata_int_list)


    def listener_callback_magnetic(self, data):
        """
        磁条消息回调
        """
        magnetic_data = data.magneticlist
        self.local_nav.magnetic_msg_callback(magnetic_data)

    def listener_callback_sod(self, data):
        obstacle_dis = data.ranges
        obstaclenumber = data.number
        self.local_nav.sod_msg_callback(obstacle_dis,obstaclenumber)
        
    def listener_callback_rfid(self, data):
        rfid_msg = int(data.data)
        self.get_logger().error("rfid_msg {}".format(rfid_msg))
        self.local_nav.rfid_msg_callback(rfid_msg)


    def Global_Path_Plan_f(self, startnode, endnode):
        while rclpy.ok() and self.client_.wait_for_service(1) == False:
            self.get_logger().info(f"等待服务端上线....")
        request = GlobalPathPlanningInterface.Request()
        request.startpoint = int(startnode)
        request.endpoint = int(endnode)
        self.client_.call_async(request).add_done_callback(self.Global_Path_Plan_result_callback)


    def Global_Path_Plan_result_callback(self, result_future):
        response = result_future.result()
        if self.local_nav.nav_ing == False:
            # self.get_logger().info('sudata.datab_t gpp : "%f"' % data.timestamp)
            # [0]start_node [1]start_node_x [1]start_node_y
            startpoint = response.startpoint
            # [0]end_node [1]end_node_x [1]end_node_y
            endpoint = response.endpoint
            # a = [::4] a[0]node_x a[1]node_Y a[2]speed a[3]angle
            routedatas = response.routedata
            self.local_nav.global_path_msg_callback(startpoint, endpoint, routedatas)
            threading.Thread(target=self.local_nav.run, args=()).start()


    def listener_callback_gpp(self, data):
        if self.local_nav.nav_ing == False:
            # self.get_logger().info('sub_t gpp : "%f"' % data.timestamp)
            # [0]start_node [1]start_node_x [1]start_node_y
            startpoint = data.startpoint
            # [0]end_node [1]end_node_x [1]end_node_y
            endpoint = data.endpoint
            # a = [::6] a[0]node a[1]node_x a[2]node_Y a[3]speed a[4]angle [5]ma_st_node [6]magnetic_state
            routedatas = data.routedata
            code = data.code
            if code:
                self.get_logger().debug("routedatas {}".format(routedatas))
                self.local_nav.global_path_msg_callback(startpoint, endpoint, routedatas)
                threading.Thread(target=self.local_nav.run, args=()).start()


    def listener_callback_fsd(self, data):
        longitude = data.longitude
        latitude = data.latitude
        obstaclenumber = data.obstaclenumber
        obstacledatas = data.obstacledata
        yaw_angle = data.yaw
        # self.get_logger().info("航向角{} obstaclenumber {}".format(yaw_angle,obstaclenumber))
        if self.local_nav.Yaw_error == None:
            self.local_nav.Yaw_error = yaw_angle
        # yaw_angle = self.local_nav.imuYaw_to_mapYaw(yaw_angle)
        # self.get_logger().info("地图偏航校{}".format(yaw_angle))
        self.local_nav.fusion_msg_callback(longitude, latitude, obstaclenumber, obstacledatas, yaw_angle)


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("magnetic_local_path_planning")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
