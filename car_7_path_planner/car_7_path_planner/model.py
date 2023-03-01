import sqlite3
from datetime import date
import os
import time
from car_setting.car_setting import LOCAL_DB

from peewee import Model, CharField, DateField, BooleanField, SmallIntegerField, SqliteDatabase, ForeignKeyField

#https://geek-docs.com/python/python-tutorial/python-peewee.html


#数据库路径
# db = SqliteDatabase('/home/nvidia/mt_ws/src/test1.db')
db = SqliteDatabase(LOCAL_DB)


class Node_data(Model):
    node = SmallIntegerField(primary_key=True)
    pose_x = SmallIntegerField()
    pose_y = SmallIntegerField()
    class Meta:
        database = db

class Edge_data(Model):
    node1 = ForeignKeyField(Node_data)
    node2 =  ForeignKeyField(Node_data)
    weight = SmallIntegerField()
    speed = SmallIntegerField()
    angle = SmallIntegerField()
    magnetic_state = SmallIntegerField()
    class Meta:
        database = db

def create_data():
    Node_data.create_table()
    Edge_data.create_table()

def add_node_data():
    node_data = Node_data.create(node=1, pose_x=3, pose_y=1)
    node_data.save()

def add_edge_data():
    edge_data = Edge_data.create(node=1, pose_x=3, pose_y=1)
    edge_data.save()

def get_node_datas():
    node_datas = Node_data.select()
    return node_datas

def get_edge_datas():
    edge_datas = Edge_data.select()
    return edge_datas

def get_edge_angle(node):
    """
    通过节点查询边中到达位置的角度
    :param node:
    :return:
    """
    edge_angle = Edge_data.select().where(Edge_data.node2 == node)
    return edge_angle[0].angle

def get_edge_magnetic_state(node):
    """
    通过节点查询边中到出发时的磁条策略
    :param node:
    :return:
    """
    edge_angle = Edge_data.select().where(Edge_data.node1 == node)
    return edge_angle[0].magnetic_state

def get_node_pose(node):
    """
    通过节点id返回节点属性列表
    :param node:
    :return:节点id,节点x,节点y
    """
    result = Node_data.select().where(Node_data.node == node)
    if len(result)>0:
        return [result[0].node,result[0].pose_x/1000,result[0].pose_y/1000]
    else:
        return -1

def nodes_is_map(node1,node2):
    result1 = Node_data.select().where(Node_data.node == node1)
    result2 = Node_data.select().where(Node_data.node == node2)
    if len(result1)>0 and len(result2)>0:
        return True
    else:
        return False

if __name__ == '__main__':
    # pass
    # create_data()
    # add_node_data()
    a_l = get_node_datas()
    for i in a_l:
        print(i)

