#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: 局部路径规划
"""

from turtle import pu
from car_setting.car_setting import CARSPEED_DEAFULT, CARTUEN_DEAFULT ,CARSPEED_STOP,CARGEAR_R ,CARGEAR_N
import rclpy        
import time                           
from rclpy.node import Node                     
from car_interfaces.msg import MagneticLocalPathPlanningInterface, PidParameterInterface
from car_interfaces.msg import FusionInterface
from car_interfaces.msg import PidInterface
"""
@topic: fusion_data FusionInterface sub


@topic: local_path_planning_data sub
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

@topic: pid_data sub
        float32     timestamp               # 数据帧时间戳
        float32     velocity                # 车辆速度m/s
        float32     angle                   # 车辆转角，方向盘转角 
        uint8       gear                    # 01:驻车:02:空挡(N):03:前进(D):04:后退(R):05:无效:
        uint8       throttle_percentage     # 油门踏板开度(0-100%)   取值:0-100
        uint8       braking_percentage      # 刹车踏板开度(0-100%) 取值: 0-100
        float32     process_time            # 进程处理时间

"""


class PIDController():
    def __init__(self):
        self.param = {"kp": 0.5, "ki": 0.0, "kd":2}
        self.limit = { "xmax": 90, "xmin": -90, "wmax": 30, "wmin": -30, "eimax": 100}
        self.error = { "xe": [0, 0, 0], "we": [0, 0, 0], "xei": 0, "wei": 0}
        self.pos = {
            "now": {"x": 0, "w": 0},
            'tgt': {"x": 0, "w": 0}
        }
        self.out = {"x": 0.0, "w": 0.0}

    def set_param(self,p,i,d):
        self.param = {"kp": p, "ki": i, "kd": d}

    def error_calc(self, pos, limit):
        """
        :param pos: pos['now']['x']实际距离 pos['now']['w'] 实际角度  pos['tgt']['x']理论距离  pos['tgt']['w']理论角度
        :param limit:
        :return:
        """
        error = self.error
        error["xe"][2] = error["xe"][1]
        error["xe"][1] = error["xe"][0]
        error["xe"][0] = pos["tgt"]["x"] - pos["now"]["x"]

        error["we"][2] = error["we"][1]
        error["we"][1] = error["we"][0]
        error["we"][0] = pos["tgt"]["w"] - pos["now"]["w"]

        error["xei"] = error["xei"] + error["xe"][0]
        error["wei"] = error["wei"] + error["we"][0]
        error["xei"] = error["xei"] if error["xei"] < limit["eimax"] else limit["eimax"]
        error["wei"] = error["wei"] if error["wei"] < limit["eimax"] else limit["eimax"]

    #Δu(k)=Kp[e(k)-e(k -1)]+Ki∑e(k)+Kd[e(k)-2e(k-1)+e(k -2)]
    def pid_calc(self, param, error, limit):
        out = {"x": error["xe"][0] * param["kp"] +
                    error["xei"] * param["ki"] +
                    (error["xe"][0] - 2 * error["xe"][1] + error["xe"][2]) * param["kd"],
               "w": error["xe"][0] * param["kp"] +
                    error["xei"] * param["ki"] +
                    (error["xe"][0] - 2 * error["xe"][1] + error["xe"][2]) * param["kd"]}

        out["x"] = out["x"] if out["w"] < limit["xmax"] else limit["xmax"]
        out["x"] = out["x"] if out["w"] > limit["xmin"] else limit["xmin"]

        out["w"] = out["w"] if out["w"] < limit["wmax"] else limit["wmax"]
        out["w"] = out["w"] if out["w"] > limit["wmin"] else limit["wmin"]
        return out

    def pid_controller(self, pos, datain):
        self.pos = pos
        self.error_calc(self.pos, self.limit)
        self.out = self.pid_calc(self.param, self.error, self.limit)
        # print("pid out x:{}  w:{}".format(self.out["x"],self.out["w"]))
        datain["x"] += self.out["x"]
        datain["w"] += self.out["w"]
        return datain



class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.pidController = PIDController()
        self.pub = self.create_publisher(
            PidInterface , "pid_data", 1)
        self.magnetic_local_path_sub = self.create_subscription(
            MagneticLocalPathPlanningInterface,
            'magnetic_local_path_planning_data',
            self.magnetic_local_path_local, 1)
        self.sub_pid = self.create_subscription(PidParameterInterface, "net_pid_data",
                                                self.listener_callback_net_pid, 10)



        #临时停车 储存速度及舵机角度
        self.stop_car_speed_angle = []
        #
        self.old_angle = 0

    def listener_callback_net_pid(self,data):
        """
        pid回调
        :param data:
        :return:
        """
        p = data.p
        i = data.i
        d = data.d
        self.pidController.set_param(p,i,d)

    def timer_callback(self): 
        pub = PidInterface()
        pub.timestamp = time.time()
        self.pub.publish(pub)
        # self.get_logger().info('pub_c: "%f"' % pub.timestamp)

    def pid_x_correct(self,angle)->int:
        angle = 90-angle
        return angle

    def pid_x_to_car_control(self,imu_angle,angle)->int:
        # print("angle {} imu_angle {}".format(angle,imu_angle))

        if imu_angle>180:
            imu_angle-=360
        elif imu_angle<-180:
            imu_angle+=360

        # if 85>angle<90:
        turn_angle = angle - self.old_angle
        self.old_angle = angle
        # else:
        #     turn_angle = 0


        #turn_angle = angle - imu_angle

        #控制左传是大于50 右转是小于50
        # print("b_ {}".format(turn_angle))
        return 50 - turn_angle*4

    def get_distance(self,local_pose,target_pose)->float:
        result_dis = pow(pow(local_pose[0] - target_pose[0], 2) + pow(local_pose[1] - target_pose[1], 2), 0.5)
        return result_dis

    def magnetic_local_path_local(self,data):
        """
        磁条局部路进规划规划回调
        :param data:
        :return:
        """
        pub = PidInterface()
        pub.timestamp = time.time()
        pub.gear = int(CARGEAR_N)
        car_stop = data.running_state
        # print("car_stop",car_stop)
        if car_stop == 1:
            pub.gear = int(CARGEAR_R)
            pub.velocity = float(CARSPEED_STOP)
            pub.angle = float(CARTUEN_DEAFULT)
        elif car_stop == 2:
            try:
                pub.gear = int(CARGEAR_R)
                pub.velocity = float(self.stop_car_speed_angle[0])
                pub.angle = float(self.stop_car_speed_angle[1])
            except Exception as e:
                print("-----e {}".format(e))
                return
        else:
            centeroffset = data.centeroffset
            # centeroffset = centeroffset*5.5
            datain = {"x": 98, "w": centeroffset*2.8125}

            """
            self.pos = {
                "now": {"x": 0, "w": 0, },
                'tgt': {"x": 0, "w": 0, }
            }
            """
            pos = {
                # 当前 x距离 当前位置 最终目标位置  w 现在imu的实际角度
                "now": {'x': -5, 'w': centeroffset},
                # 目标 x距离 虚拟位置位置 最终目标位置  w算出来的目标角度
                "tgt": {'x': 0, 'w': 0}
            }
            datain = self.pidController.pid_controller(pos, datain)

            speed = datain['x']
            angle = datain['w']

            #修改速度
            pub.velocity = float(CARSPEED_DEAFULT)
            #修改舵机方向
            pub.angle = float(CARTUEN_DEAFULT+angle)
        # print("pub {}".format(pub))
        self.pub.publish(pub)

    def listener_callback_magnetic_local(self, data):
        """
        定位局部路进规划规划回调
        :param data:
        :return:
        """
        pub = PidInterface()
        pub.timestamp = time.time()
        pub.gear = int(CARGEAR_N)
        car_stop = data.running_state
        if car_stop == 1:
            pub.gear = int(CARGEAR_R)
            pub.velocity = float(CARSPEED_STOP)
            pub.angle = float(CARTUEN_DEAFULT)
        elif car_stop == 2:
            try:
                pub.gear = int(CARGEAR_R)
                pub.velocity = float(self.stop_car_speed_angle[0])
                pub.angle = float(self.stop_car_speed_angle[1])
            except Exception as e:
                print("-----e {}".format(e))
                return
        else:
            # 控制速度
            speed = data.speed
            # 控制角度
            angle = data.angle
            # print("speed {} angle {}".format(speed,angle))
            # process_time为imu的偏航角
            imu_angle = data.process_time
            # print("imu_angle {}".format(imu_angle))
            #
            startpoint = data.startpoint
            startpoint_pose = (startpoint[0],startpoint[1])

            endpoint = data.endpoint
            endpoint_pose = (endpoint[0], endpoint[1])

            local_pose = (data.longitude,data.longitude)
            datain = {"x":speed,"w":angle}
            # print("angle {} ".format(angle))

            """
            self.pos = {
                "now": {"x": 0, "w": 0, },
                'tgt': {"x": 0, "w": 0, }
            }
            """
            pos = {
                #当前 x距离 当前位置 最终目标位置  w 现在imu的实际角度
                "now": {'x': self.get_distance(local_pose,endpoint_pose),'w':imu_angle},
                #目标 x距离 虚拟位置位置 最终目标位置  w算出来的目标角度
                "tgt": {'x': self.get_distance(startpoint_pose,endpoint_pose),'w':angle}
                   }
            datain = self.pidController.pid_controller(pos,datain)
            speed = datain['x']
            angle = datain['w']
            pub.velocity = speed
            angle = self.pid_x_to_car_control(imu_angle,angle)
            pub.angle = angle
            self.stop_car_speed_angle = [speed,angle]
        self.get_logger().info('pub: {}'.format(pub))
        self.pub.publish(pub)



         
        



        
def main(args=None):                                
    rclpy.init(args=args)                           
    node = PublisherNode("pid_data")  
    rclpy.spin(node)                               
    node.destroy_node()                            
    rclpy.shutdown()      
