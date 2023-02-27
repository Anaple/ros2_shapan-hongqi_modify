
import rclpy
from rclpy.node import Node


import json
import socket
import struct
import threading
import time
from std_msgs.msg import String


class Socket_Server(Node):
    def __init__(self,port):
        self.get_logger().info('连接wait')
        self._heart_data = bytes.fromhex("99")
        socket_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket_.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.socket_=socket_
        host = "0.0.0.0"
        socket_.bind((host, port))
        socket_.listen(1)
        # self._client_socket, address = socket_.accept()
        # logger.info(str(address) +self.class_name()+ '连接成功')
        # self.socket_connect=True
        #初始化发布器
        self.uwb_publisher = self.create_publisher(String, 'uwb_data', 10)
        self.uwb_data=String()
        threading.Thread(target=self._heartbeat_threading).start()
        threading.Thread(target=self._reconnection_threading).start()
        threading.Thread(target=self.__threading_recv).start()

    @classmethod
    def class_name(cls):
        return str(cls.__name__)
    def _heartbeat_threading(self):
        while True:
            if self.socket_connect:
                try:
                    self._client_socket.send(self._heart_data)
                except BrokenPipeError :
                    self.get_logger().error(self.class_name()+"BrokenPipeError")
                    self.socket_connect=False
                except ConnectionResetError:
                    self.get_logger().error(self.class_name() + "ConnectionResetError")
                    self.socket_connect = False
                time.sleep(5)

    def _reconnection_threading(self):
        while True:
            if not self.socket_connect:
                self.get_logger().info(self.class_name()+"等待客户端重新连接")
                self._client_socket, address = self.socket_.accept()
                self.get_logger().info(self.class_name()+str(address)+'重新连接成功')
                self.socket_connect=True
            time.sleep(1)
    def __threading_recv(self):
        """
        {
        "Time": "114138665",
        "TagID": "0006",
        "Seq": "104",
        "Mask": "0F",
        "Dimen": "3",
        "Coord_valid": "1",
        "Coord": {
            "x": "3.48",
            "y": "0.61",
            "z": "1.82"
            }
        }
        :return:
        """
        while True:
            if self.socket_connect:
                try:
                    data = self._client_socket.recv(1024)
                    if data:
                        data = data.decode()
                        data = json.loads(data)
                        #解析json数据
                        coord_valid = data['Coord_valid']
                        TagID = data['TagID']
                        if coord_valid == '0':
                            x = data['Coord']['x']
                            y = data['Coord']['y']
                            z = data['Coord']['z']
                            self.uwb_data.data = f"{TagID},{x},{y},{z}"
                            self.uwb_publisher.publish(self.uwb_data.data)
                        else:
                            print("定位失败")
                except ConnectionResetError:
                    self.get_logger().error(self.class_name() + "ConnectionResetError")
                    self.socket_connect = False
                except json.decoder.JSONDecodeError:
                    self.get_logger().error(self.class_name() + "json.decoder.JSONDecodeError")
                    self.socket_connect = False
                except Exception as e:
                    self.get_logger().error(self.class_name() + str(e))
                    self.socket_connect = False
            time.sleep(0.01)

def main():
    rclpy.init()
    node = Socket_Server(5000)
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()


