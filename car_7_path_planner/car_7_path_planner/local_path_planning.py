#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: 局部路径规划
"""
import threading
import math
import rclpy        
import time                           
from rclpy.node import Node                     
from car_interfaces.msg import GlobalPathPlanningInterface
from car_interfaces.msg import LocalPathPlanningInterface
from car_interfaces.msg import FusionInterface

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

@topic: fusion_data FusionInterface sub


@topic: local_path_planning_data pub
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
    def __init__(self,pub):
        #car_pub 发布节点
        self.pub = pub
        #global_path_msg  接受全局路径规划消息
        #开始节点位置
        self.startpoint = None
        #结束节点位置
        self.endpoint = None
        #规划路径的节点
        self.routedata = None
        #是否在执行导航
        self.nav_ing = False
        #车辆是否在运动
        self.car_run = False
        #车辆的位置信息
        self.longitude = None
        self.latitude = None
        self.yaw_angle = None
        #障碍物信息
        self.obstaclenumber = 0
        self.obstacledatas = []

        #stop_car_distance 遇到障碍物停车阈值
        self.stop_car_distance = 0.2
        #deviation 定位误差阈值
        self.deviation = 0.1

        #将小车垂直地图log方向放置,查看偏航角
        self.Yaw_error =None

        #虚拟点的数量
        self.expand_num = 5


    def imuYaw_to_mapYaw(self,imuYaw):
        """
        imu偏航校转地图的转角
        :param imuYaw:
        :return:
        """
        imuYaw = imuYaw-self.Yaw_error
        if imuYaw < 0:
            imuYaw += 360
        return imuYaw

    def list_of_groups(self,init_list, children_list_len):
        """
        收到的ros msg消息 以协议形式切割为4位一组的列表
        :param init_list: 以为长度的列表
        :param children_list_len: 切片长度
        :return: 输出二维数组
        """
        list_of_groups = zip(*(iter(init_list),) * children_list_len)
        end_list = [list(i) for i in list_of_groups]
        count = len(init_list) % children_list_len
        end_list.append(init_list[-count:]) if count != 0 else end_list
        return end_list

    def stop_car(self):
        """
        发送停车消息 消息为ros下msg speed 为0停车 angle为50，舵机回正
        :return: 无
        """
        self.car_run = False
        pub_msg = LocalPathPlanningInterface()
        pub_msg.timestamp = float(1)
        print("stop_car")
        self.pub.publish(pub_msg)

    def restore_car(self):
        """
        障碍物小时,车辆恢复运动
        :return: 无
        """
        self.car_run = True
        pub_msg = LocalPathPlanningInterface()
        pub_msg.timestamp = float(2)
        print("restore_car")
        self.pub.publish(pub_msg)

    def movement_car(self,speed,angle,yaw_angle,startpoint,endpoint):
        """
        发送车辆控制消息
        :param speed: 车辆速度
        :param angle: 车辆转弯角度
        :return: 无
        """
        self.car_run = True
        pub_msg = LocalPathPlanningInterface()
        pub_msg.timestamp = float(3)
        pub_msg.speed = float(speed)
        pub_msg.angle = float(angle)
        pub_msg.startpoint = startpoint
        pub_msg.endpoint = endpoint
        pub_msg.process_time = yaw_angle
        pub_msg.longitude = self.longitude
        pub_msg.latitude = self.latitude
        print("movement_car")
        self.pub.publish(pub_msg)

    def global_path_msg_callback(self,startpoint,endpoint,routedata):
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
        # a = [::4] a[0]node_x a[1]node_Y a[2]speed a[3]angle
        self.routedata = routedata


    def pose_judge(self,pose1,pose2)->bool:
        """
        输入两个位置，通过阈值判断是不是是否属于同一点
        :param pose1: 点1
        :param pose2: 点2
        :return:
        """
        # print(pose1,pose2)
        deviation_x = abs(pose1[0] - pose2[0])
        deviation_y = abs(pose1[1] - pose2[1])
        # print(deviation_x,deviation_y)
        if deviation_x < self.deviation and deviation_y < self.deviation:
            return True
        else:
            return False

    def node_expand(self,start_pose,end_pose):
        """
        点位扩充
        :param start_pose: 开始位置
        :param end_pose: 结束位置
        :return:
        """

        start_x = start_pose[0]
        start_y = start_pose[1]
        start_speed = start_pose[2]
        start_angle = start_pose[3]


        end_x = end_pose[0]
        end_y = end_pose[1]
        end_speed = end_pose[2]
        end_angle = end_pose[3]
        angle = start_angle - end_angle

        if angle>180:
            angle-=360
        elif angle<-180:
            angle+=360


        inter_x = (start_x - end_x) / self.expand_num
        inter_y =( start_y - end_y) / self.expand_num
        inter_angle = (angle) / self.expand_num
        # start_angle+=inter_angle
        result = [[float(start_x),float(start_y),start_speed,float(start_angle)]]

        for i in range(self.expand_num):
            start_x -= inter_x
            start_y -= inter_y
            start_angle += inter_angle
            result.append([start_x,start_y , start_speed, start_angle])

        for i in range(len(result)):
            if i!=len(result)-1:
                result[i][3]+=inter_angle


        return result


    def execute_nav(self):
        """
        开始执行导航
        :return:
        """
        self.nav_ing = True
        print("开始局部导航")
        print("等待融合数据")
        #阻塞等待融合数据
        while self.latitude==None or self.Yaw_error==None:
            pass

        while True:
            #判断是否在导航开始位置
            if self.pose_judge([self.longitude ,self.latitude],[self.startpoint[1],self.startpoint[2]]):
                nav_msg_list = self.list_of_groups(self.routedata,4)
                # print("endpoint {} {}".format(self.endpoint[1],self.endpoint[2]))
                # print("nav_msg_list  ",len(nav_msg_list),nav_msg_list)
                #增加虚拟点位
                nav_msg_lists = []
                print(nav_msg_list)
                for i in range(len(nav_msg_list)-1):
                    nav_msg_lists += self.node_expand(nav_msg_list[i],nav_msg_list[i+1])

                print(len(nav_msg_lists),nav_msg_lists)

                for i in range(len(nav_msg_lists)-1):
                    node_pose_x = nav_msg_lists[i][0]
                    node_pose_y = nav_msg_lists[i][1]
                    print("阻塞开始,运动点位 {} ".format([node_pose_x, node_pose_y]))
                    while self.pose_judge([self.longitude, self.latitude], [node_pose_x, node_pose_y])==False:
                        time.sleep(0.05)
                    print("阻塞结束 ，车辆位置{} ".format([self.longitude, self.latitude]))
                    index_pose = self.expand_num+1
                    if i % index_pose == 0 and i+index_pose != len(nav_msg_lists):
                        #更新小目标位置
                        endpoint = [nav_msg_lists[i+index_pose][0],nav_msg_lists[i+index_pose][1]]
                    startpoint = [nav_msg_lists[i][0],nav_msg_lists[i][1]]
                    self.movement_car(nav_msg_lists[i][2],nav_msg_lists[i][3],self.yaw_angle,startpoint,endpoint)
                    print("下发运动指令 {} ".format(nav_msg_lists[i]))
                print("开始阻塞，到终点结束{}".format(self.endpoint))
                while self.pose_judge([self.longitude, self.latitude], [self.endpoint[1], self.endpoint[2]])==False:
                    time.sleep(0.05)
                print("导航结束，车辆停止")
                self.stop_car()
                #运动结束
                self.nav_ing = False
                return
            else:
                pass
                # print("当前位置->{}，不在导航起始位置->{}".format(str(self.longitude) + " " + str(self.latitude),self.startpoint))



    def fusion_msg_callback(self,longitude,latitude,obstaclenumber,obstacledatas,yaw_angle):
        """
        融合节点消息回调
        :param longitude:x uwb小车位置x
        :param latitude:y uwb小车位置y
        :param obstaclenumber: 障碍物数量
        :param obstacledatas: 障碍物数据
        :return:
        """
        self.longitude = longitude
        self.latitude = latitude
        self.yaw_angle = yaw_angle
        # if obstaclenumber > 0:
        #     for i in range(obstaclenumber):
        #         #接收到融合数据的障碍物信息后，如果小车在执行导航
        #         if self.nav_ing:
        #             #判断小车运动状态及障碍物距离信息，控制小车运动
        #             if self.car_run and obstacledatas[i] < self.stop_car_distance:
        #                 # print("遇障碍物停止")
        #                 # print("obstacledatas[i] {}".format(obstacledatas[i]))
        #                 self.stop_car()
        # else:
        #     if not self.car_run and self.nav_ing:
        #         # 当车辆属于停止，障碍大于阈值 让车辆恢复运动
        #         # print("障碍物消失，恢复运动")
        #         self.restore_car()







class PublisherNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.pub = self.create_publisher(
            LocalPathPlanningInterface , "local_path_planning_data", 10)
        self.local_nav = Local_nav(self.pub)
        self.sub = self.create_subscription(
            GlobalPathPlanningInterface, 
            'global_path_planning_data', 
            self.listener_callback_gpp, 10)
        self.sub = self.create_subscription(
            FusionInterface, 
            'fusion_data', 
            self.listener_callback_fsd, 10)

        


    def listener_callback_gpp(self, data):
        if self.local_nav.nav_ing==False:
            # self.get_logger().info('sub_t gpp : "%f"' % data.timestamp)
            # [0]start_node [1]start_node_x [1]start_node_y
            startpoint = data.startpoint
            # [0]end_node [1]end_node_x [1]end_node_y
            endpoint = data.endpoint
            # a = [::4] a[0]node_x a[1]node_Y a[2]speed a[3]angle
            routedatas = data.routedata
            self.local_nav.global_path_msg_callback(startpoint,endpoint,routedatas)
            if not self.local_nav.nav_ing:
                threading.Thread(target=self.local_nav.execute_nav,args=()).start()



    def listener_callback_fsd(self, data):
        longitude = data.longitude
        latitude = data.latitude
        obstaclenumber = data.obstaclenumber
        obstacledatas = data.obstacledata
        yaw_angle = data.yaw
        # self.get_logger().info("航向角{} obstaclenumber {}".format(yaw_angle,obstaclenumber))
        if self.local_nav.Yaw_error==None:
            self.local_nav.Yaw_error = yaw_angle
        yaw_angle = self.local_nav.imuYaw_to_mapYaw(yaw_angle)
        # self.get_logger().info("地图偏航校{}".format(yaw_angle))
        self.local_nav.fusion_msg_callback(longitude,latitude,obstaclenumber,obstacledatas,yaw_angle)




def main(args=None):                                
    rclpy.init(args=args)                           
    node = PublisherNode("local_path_planning")  
    rclpy.spin(node)                               
    node.destroy_node()                            
    rclpy.shutdown()      
