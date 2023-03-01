#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: uwb 
"""
from car_setting.car_setting import TTY_UWB
import rclpy
import transforms3d
from rclpy.node import Node
import cv2
import math
from tkinter import END
import numpy as np

from geometry_msgs.msg import Pose,TransformStamped
from car_interfaces.msg import GPSInterface
import serial
import sympy
import time
from tf2_ros import StaticTransformBroadcaster

"""
@topic: gps_data pub
        float32  timestamp      # 时间戳
        uint8    id             # 导航ID
        float32  yaw            # 偏航角0-359.99
        float32  pitch          # 俯仰角90-90
        float32  roll           # 横滚角-180-180
        float32  wx             # 角速度x
        float32  wy             # 角速度y
        float32  wz             # 角速度z
        float32  ax             # 加速度x
        float32  ay             # 加速度y
        float32  az             # 加速度z
        float32  longitude      # 经度
        float32  latitude       # 纬度
        float32  height         # 高度
        float32  eastvelocity   # 东向速度
        float32  northvelocity  # 北向速度
        float32  skyvelocity    # 天向速度
        float32  process_time   # 进程处理时间
"""
def calc_abc_from_line_2d(P1,P2):
    x0=P1.x
    y0=P1.y
    x1=P2.x
    y1=P2.y
    a = y0-y1
    b = x1-x0
    c = x0*y1-x1*y0
    return a, b, c
def get_line_cross_point(P1,P2,P3,P4):
    a0, b0, c0 = calc_abc_from_line_2d(P1,P2)
    a1, b1, c1 = calc_abc_from_line_2d(P3,P4)
    D = a0*b1-a1*b0
    if D==0:
        return None
    x = (b0*c1-b1*c0)/D
    y = (a1*c0-a0*c1)/D
    return Point(x,y)
class Point():
    def __init__(self,x,y):
        self.x=x
        self.y=y
def dis(P1,P2):
    return math.sqrt((P2.x-P1.x) ** 2 + (P2.y-P1.y) ** 2)

def Incenter(a,b,c):
    dA=dis(b,c)
    dB=dis(a,c)
    dC=dis(a,b)
    S=dA+dB+dC
    x=(dA*a.x+dB*b.x+dC*c.x)/S
    y=(dA*a.y+dB*b.y+dC*c.y)/S
    return Point(x,y)
class UWB_Node(Node):

    def duStation_s(self, p_pose, r_list):
        muxpoint = [[0, 0], [0, 0], [0, 0]]
        # 所有交点位置
        muxpoint[0] = self.getIntersection(p_pose[0], r_list[0], p_pose[1], r_list[1])
        muxpoint[1] = self.getIntersection(p_pose[0], r_list[0], p_pose[2], r_list[2])
        muxpoint[2] = self.getIntersection(p_pose[1], r_list[1], p_pose[2], r_list[2])
        Pab1 = Point(muxpoint[0][0][0], muxpoint[0][0][1])
        Pab2=Point(muxpoint[0][1][0], muxpoint[0][1][1])
        Pac1=Point(muxpoint[1][0][0], muxpoint[1][0][1])
        Pac2=Point(muxpoint[1][1][0], muxpoint[1][1][1])
        Pbc1=Point(muxpoint[2][0][0], muxpoint[2][0][1])
        Pbc2=Point(muxpoint[2][1][0], muxpoint[2][1][1])
        A=get_line_cross_point(Pab1,Pab2,Pac1,Pac2)
        B= get_line_cross_point(Pab1, Pab2, Pbc1, Pbc2)
        C = get_line_cross_point(Pbc1, Pbc2, Pac1, Pac2)
        veryveryverygood_value=Incenter(A,B,C)
        return [veryveryverygood_value.x,veryveryverygood_value.y]
    def __init__(self, name):
        super().__init__(name)
        self.anchor = [0.0] * 3
        self.station = [[0, 0], [0, 3.30], [2.36, 0]]
        self.is_show_img = True
        self.pub = self.create_publisher(GPSInterface, "gps_data", 10)
        self.serial = serial.Serial(TTY_UWB, 115200, timeout=0.5)

        self.static_tf = StaticTransformBroadcaster(self)

        static_tf_msg = TransformStamped()
        static_tf_msg.header.stamp = self.get_clock().now().to_msg()
        static_tf_msg.header.frame_id = "base_link"
        static_tf_msg.child_frame_id = "uwb_location"
        static_tf_msg.transform.translation.x = 0.03
        static_tf_msg.transform.translation.y = -0.05
        static_tf_msg.transform.translation.z = 0.08
        qua = transforms3d.euler.euler2quat(0.0,0.0,3.141592653572,"sxyz")
        static_tf_msg.transform.rotation.x = qua[0]
        static_tf_msg.transform.rotation.y = qua[1]
        static_tf_msg.transform.rotation.z = qua[2]
        static_tf_msg.transform.rotation.w = qua[3]
        self.static_tf.sendTransform(static_tf_msg)


    def getStationDisData(self, datahex):
        offset = 6
        for i in range(len(datahex)):
            if datahex[i] == 109 and i < 6:
                offset = 6 + i
        distance = [0] * 3
        distance[0] = (datahex[offset + 1] << 8 | datahex[offset + 0]) / 100
        distance[1] = (datahex[offset + 3] << 8 | datahex[offset + 2]) / 100
        distance[2] = (datahex[offset + 5] << 8 | datahex[offset + 4]) / 100
        return distance

    def getIntersection(self, p1, r1, p2, r2):
        x = p1[0]
        y = p1[1]
        R = r1
        a = p2[0]
        b = p2[1]
        S = r2
        d = math.sqrt((abs(a-x))**2 + (abs(b-y))**2)
        c = [[0, 0], [0, 0]]
        if d > (R + S) or d < (abs(R - S)):
            pass
            # print("dis error")
        elif d == 0 and R==S :
            pass
            # print("circles center error")
        else:
            A = (R**2 - S**2 + d**2) / (2 * d)
            h = math.sqrt(R**2 - A**2)
            x2 = x + A * (a-x)/d
            y2 = y + A * (b-y)/d
            x3 = round(x2 - h * (b - y) / d, 2)
            y3 = round(y2 + h * (a - x) / d, 2)
            x4 = round(x2 + h * (b - y) / d, 2)
            y4 = round(y2 - h * (a - x) / d, 2)
            c[0] = np.array([x3, y3])
            c[1] = np.array([x4, y4])
        return c

    def muxpoint_near_judgement(self, pos_a, pos_b):
        if (abs(pos_a[0] - pos_b[0])) < (abs(pos_a[0])) * 1.5 and (abs(pos_a[1] - pos_b[1])) < (abs(pos_a[1])) * 1.5 and pos_a[0]!=0 and pos_a[1]!=0 :
            return True
        else:
            return False

    def duStation_t(self, p_pose, r_list):
        # 所有交点位置
        muxpoint = self.getIntersection(p_pose[0], r_list[0], p_pose[1], r_list[1])
        muxpoint += self.getIntersection(p_pose[0], r_list[0], p_pose[2], r_list[2])
        muxpoint += self.getIntersection(p_pose[1], r_list[1], p_pose[2], r_list[2])
        for i in range(len(muxpoint)-1):
            mux_near_point_0 = muxpoint[i]
            mux_near_point_1 = muxpoint[i + 1]
            if self.muxpoint_near_judgement(mux_near_point_0,mux_near_point_1):
                #self.get_logger().info("muxpoint---->  {} muxpoint----> {}".format(mux_near_point_0,mux_near_point_1))
                mux_near_point = [0] * 2
                mux_near_point[0] = (mux_near_point_0[0] + mux_near_point_1[0]) / 2
                mux_near_point[1] = (mux_near_point_0[1] + mux_near_point_1[1]) / 2
                #self.get_logger().info("mux_near_point---->  {} ".format(mux_near_point))
                return mux_near_point

        return [0, 0]

    def duStation_d(self, p_pose, r_list):
        muxpoint = [[0, 0], [0, 0], [0, 0]]
        # 所有交点位置
        muxpoint[0] = self.getIntersection(p_pose[0], r_list[0], p_pose[1], r_list[1])
        muxpoint[1] = self.getIntersection(p_pose[0], r_list[0], p_pose[2], r_list[2])
        muxpoint[2] = self.getIntersection(p_pose[1], r_list[1], p_pose[2], r_list[2])

        mpdis = [0.0] * 12
        mpindex = [[0, 1], [0, 2], [1, 2]]
        minsdis = [[1000, 0],[1000,0]]
        pos = [0, 0]
        for i in range(12):
            pos1 = muxpoint[mpindex[int(i / 4)][0]][int((i % 4) / 2)]
            pos2 = muxpoint[mpindex[int(i / 4)][1]][int((i % 4) % 2)]
            mpdis[i] = math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)
            if mpdis[i] < minsdis[0][0] and mpdis[i] > 0:
                minsdis[0][0] = mpdis[i]
                minsdis[0][1] = i
                mpdis[i] = 1000

        for i in range(12):
            if mpdis[i] < minsdis[1][0] and mpdis[i] > 0:
                minsdis[1][0] = mpdis[i]
                minsdis[1][1] = i

  
        # print("minsdis:{}---mpdis: {}".format(minsdis,mpdis))
        pos_a = muxpoint[mpindex[int(minsdis[0][1] / 4)][0]][int((minsdis[0][1] % 4) / 2)]
        pos_b = muxpoint[mpindex[int(minsdis[0][1] / 4)][1]][int((minsdis[0][1] % 4) % 2)]
        pos_c = muxpoint[mpindex[int(minsdis[1][1] / 4)][0]][int((minsdis[1][1] % 4) / 2)]
        pos_d = muxpoint[mpindex[int(minsdis[1][1] / 4)][1]][int((minsdis[1][1] % 4) % 2)]
        # print("pos_a:{}---pos_b: {}---pos_c: {}---pos_d: {}".format(pos_a,pos_a,pos_c,pos_d))
        pos[0] = (pos_a[0] + pos_b[0] + pos_c[0] + pos_d[0]) / 4
        pos[1] = (pos_a[1] + pos_b[1] + pos_c[1] + pos_d[1]) / 4
        return pos



    def triposition(self,station,anchor ):
        xa = station[0][0]
        ya = station[1][0]
        da = station[2][0]
        xb = station[0][1]
        yb = station[1][1]
        db = station[2][1]
        xc = anchor[0]
        yc = anchor[1]
        dc = anchor[2]

        x, y = sympy.symbols('x y')
        f1 = 2 * x * (xa - xc) + np.square(xc) - np.square(xa) + 2 * y * (ya - yc) + np.square(yc) - np.square(ya) - (
                    np.square(dc) - np.square(da))
        f2 = 2 * x * (xb - xc) + np.square(xc) - np.square(xb) + 2 * y * (yb - yc) + np.square(yc) - np.square(yb) - (
                    np.square(dc) - np.square(db))
        result = sympy.solve([f1, f2], [x, y])
        locx, locy = result[x], result[y]
        return [locx, locy]
        
    
    def showStationDrectingNode(self, inpt, anchor, pos):
        self.img = np.ones((400, 400, 3), dtype="uint8") * 255

        p1_ = (int(inpt[0][0] * 100), int(inpt[0][1] * 100))
        p2_ = (int(inpt[1][0] * 100), int(inpt[1][1] * 100))
        p3_ = (int(inpt[2][0] * 100), int(inpt[2][1] * 100))

        cv2.circle(self.img, p1_, int(anchor[0] * 100), 100, 3)
        cv2.circle(self.img, p2_, int(anchor[1] * 100), 100, 3)
        cv2.circle(self.img, p3_, int(anchor[2] * 100), 100, 3)

        cv2.circle(self.img, (int(pos[0] * 100), int(pos[1] * 100)), 3, (0, 255, 0), 1)
        cv2.putText(self.img,
                    "{},{}".format(round(pos[0], 2),
                     round(pos[1], 2)),
                    (int(pos[0] * 100), int(pos[1] * 100)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        cv2.namedWindow('img')
        cv2.imshow('img', self.img)
        cv2.waitKey(1)

    def getLocation(self):
        pos = [0, 0]
        try:
            rxbuf = self.serial.read(16)
            #print(rxbuf)
            self.anchor = self.getStationDisData(rxbuf)
            # pos = self.duStation_d(self.station, self.anchor)
            pos = self.duStation_s(self.station, self.anchor)

            #pos = self.triposition(self.station, self.anchor)
            self.showStationDrectingNode(self.station, self.anchor,pos)
            #self.get_logger().info("rStation {} pos {}".format(self.anchor, pos))
        except Exception as e:
            print(e)
            self.serial.close()
            print("关闭串口")
            exit()
        return pos

    def pubLocationData(self):
        pub = GPSInterface()
        pub.timestamp = time.time()
        pos = self.getLocation()
        # print(pos)
        if pos[0] != 0 and pos[1] != 0:
            pub.longitude = round(pos[0], 2)
            pub.latitude = round(pos[1], 2)
            self.pub.publish(pub)

def main(args=None):
    rclpy.init(args=args)                           
    node = UWB_Node("uwb_location")
    while(rclpy.ok()):
        node.pubLocationData()
    rclpy.spin(node)                               
    node.destroy_node()
    rclpy.shutdown()                      



if __name__ == '__main__':
    main()


    
