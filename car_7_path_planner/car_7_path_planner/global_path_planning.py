#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: kmakise
@说明: 全局路径规划
"""

import rclpy
import time

from rclpy.node import Node
from car_7_path_planner.AMGraph import Map
from std_msgs.msg import String
from car_7_path_planner.model import get_node_datas, get_edge_datas, get_node_pose, nodes_is_map, get_edge_angle, \
    get_edge_magnetic_state
from car_interfaces.msg import NetStationInterface,GlobalPathPlanningInterface,NavigationalStateInterface
from std_msgs.msg import String
from rclpy.publisher import Publisher


class Navigation():
    def __init__(self,local_rfid:int,pub:Publisher):
        self.create_map()
        self.init_parameter()
        self.local_rfid = local_rfid
        self.end_node_list = []
        self.local_plann_flag = True
        self.pub = pub


    def nav_ShortestPath(self):
        try:
            node_edge = self.map.ShortestPath(self.start_node, self.end_node)
            result = []
            for i in node_edge:
                # node_pose [0]node [1]node_x [2]node_y
                pose = get_node_pose(i["node"])
                # print(i['ma_st_node'],i['magnetic_state'])
                result.append([float(i["node"]),float(pose[1]), float(pose[2]), float(i['speed']),float(i['angle']),float(i["ma_st_node"]),float(i["magnetic_state"])])
                # node node_x node_y speed angle magnetic_state
            return result
        except Exception as e:
            return -1

    def create_map(self):
        self.map = Map()
        self.map.create_graph()
        node_datas = get_node_datas()
        for i in node_datas:
            self.map.add_node(i.node, i.pose_x, i.pose_y)

        for i in get_edge_datas():
            self.map.add_edge(i.node1.node, i.node2.node, i.weight, i.speed, i.angle,i.magnetic_state)

    def net_station_sub_callback(self, startnode, endnode):
        if nodes_is_map(startnode, endnode):
            self.start_node = startnode
            self.end_node = endnode
            self.start_pose = get_node_pose(startnode)
            self.end_pose = get_node_pose(endnode)
            return self.nav_ShortestPath()
        else:
            return -1

    def init_parameter(self):
        self.start_pose = None
        self.end_pose = None
        self.start_node = None
        self.end_node = None

    def set_end_node_list(self,end_pose):
        self.end_node_list.append(end_pose)
        if self.local_plann_flag and len(self.end_node_list)>0:
            result = self.planning(self.end_node_list.pop(0))
            # assert isinstance(Publisher,self.pub)
            self.pub.publish(result)
            self.local_plann_flag = False

    def set_local_plann_flag(self,local_plann_flag):
        self.local_plann_flag = local_plann_flag
        if self.local_plann_flag and len(self.end_node_list)>0:
            result = self.planning(self.end_node_list.pop(0))
            # assert isinstance(Publisher,self.pub)
            self.pub.publish(result)
            self.local_plann_flag = False

    def planning(self,end):
        pub_msg = GlobalPathPlanningInterface()
        if self.local_rfid==0:
            # self.get_logger().error("车辆定位实效")
            pub_msg.code = False
        else:
            startnode = self.local_rfid
            endnode = int(end)
            # self.get_logger().info("startnode {} endnode {}".format(startnode, endnode))
            result = self.net_station_sub_callback(startnode, endnode)
            if result == -1:
                # self.get_logger().error("导航的点不在地图")
                pub_msg.code = False
            elif result == -2:
                # self.get_logger().error("路径规划失败")
                pub_msg.code = False
            else:
                pub_msg.code = True
                self.nav_result = result
                pub_msg.timestamp = time.time()
                # 起点位置
                # startpoint[0] startpoint[1] startpoint[2]
                pub_msg.startpoint = [float(self.start_pose[0]), float(self.start_pose[1]),
                                      float(self.start_pose[2])]
                # 终点位置
                pub_msg.endpoint = [float(self.end_pose[0]), float(self.end_pose[1]),
                                    float(self.end_pose[2])]
                routedata = []
                for i in self.nav_result:
                    # i [0]node_x [1]node_Y [2]speed [3]angle
                    routedata += i
                # print(routedata)
                pub_msg.routedata = routedata
        return pub_msg


class PublisherNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.nav_result = None
        self.local_rfid = 1
        self.pub = self.create_publisher(GlobalPathPlanningInterface, "global_path_planning_data", 10)
        self.navigation = Navigation(self.local_rfid,self.pub)
        self.rfid_sub = self.create_subscription(String, "rfid_data", self.listener_callback_rfid, 1)
        # self.pub_global_path_planning_result = self.create_publisher(String, "global_path_planning_data", 10)

        self.sub_nav_data = self.create_subscription(NavigationalStateInterface, 'nav_data', self.listener_callback_sub_nav_data, 10)
        self.sub = self.create_subscription(NetStationInterface, 'net_station_data', self.listener_callback, 10)
        # self.add_ints_server_ = self.create_service(GlobalPathPlanningInterface, "Global_Path_Plan", self.Global_Path_Plan)
        self.get_logger().info("节点已启动：%s!" % name)

    def listener_callback_sub_nav_data(self,data):
        local_plann_flag = not data.save_flag
        self.navigation.set_local_plann_flag(local_plann_flag)

    def Global_Path_Plan(self,request, response):
        startnode = request.startpoint
        endnode = request.endpoint
        self.get_logger().info("startnode {} endnode {}".format(startnode, endnode))
        result = self.navigation.net_station_sub_callback(startnode, endnode)
        if result == -1:
            self.get_logger().error("导航的点不在地图")
            return response
        else:
            response.timestamp = time.time()
            # 起点位置
            # startpoint[0] startpoint[1] startpoint[2]
            response.startpoint = [float(self.navigation.start_pose[0]), float(self.navigation.start_pose[1]),
                                  float(self.navigation.start_pose[2])]
            # 终点位置
            response.endpoint = [float(self.navigation.end_pose[0]), float(self.navigation.end_pose[1]),
                                float(self.navigation.end_pose[2])]
            routedata = []
            for i in result:
                # i [0]node_x [1]node_Y [2]speed [3]angle [4]magnetic_state
                routedata += i

            # print(routedata)
            response.routedata = routedata
            return response

    def listener_callback_rfid(self,data):
        self.local_rfid = int(data.data)
        self.get_logger().info(f"rfid={self.local_rfid},起点更新：{self.navigation.local_rfid}")
        self.navigation.local_rfid = self.local_rfid
        self.navigation = Navigation(self.local_rfid,self.pub)



    def listener_callback(self, data):
        endnode = int(data.endpoint[0])
        self.navigation.set_end_node_list(endnode)



def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("global_path_planning")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()      