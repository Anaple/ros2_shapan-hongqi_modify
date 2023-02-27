import rclpy
from car_interfaces.msg import FusionInterface, NavigationalStateInterface
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class PublisherNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.sub_fus = self.create_subscription(FusionInterface, 'fusion_data', self.listener_callback_fsd, 10)
        self.sub_nav = self.create_subscription(NavigationalStateInterface, 'nav_data', self.listener_callback_nav, 10)
        self.save_flag = None
        self.start = 0
        self.end = 0
        self.num = 0
        self.result = []
        # my_parameter_tset = ParameterDescriptor(description='This parameter is mine!')

        self.declare_parameter('my_parameter_tset',"test")


    def listener_callback_nav(self, data):
        self.start = data.start
        self.end = data.end
        if data.save_flag:
            self.save_flag = True
            self.get_logger().info("start save")
        else:
            self.save_flag = False
            self.get_logger().info("save ...")

    def listener_callback_fsd(self, data):
        if self.num>2:
            longitude = round(data.longitude, 2)
            latitude = round(data.latitude, 2)
            height = round(data.height, 2)
            self.num = 0
            if self.save_flag == True:
                self.result.append("({},{})".format(longitude, latitude))

            elif self.save_flag == False:
                self._write_file(self.start, self.end)
                self.save_flag = None
        else:
            self.num+=1
          

    def _write_file(self, start, end):
        data = "[" + ",".join(self.result) + "]"
        self.get_logger().info(data)
        file_path ='/home/nvidia/mt_ws/src/car_4_communication/car_4_communication/point_data/{}-{}.txt'.format(start, end) 
        self.get_logger().info(file_path)
        f = open(file_path, 'w')
        f.write(data + '\n')
        self.result = []


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("point_data_bag")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
