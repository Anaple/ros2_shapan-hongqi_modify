import rclpy
from car_interfaces.msg import FusionInterface
from rclpy.node import Node


class PublisherNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)

        self.sub_fus = self.create_subscription(FusionInterface,'fusion_data',self.listener_callback_fsd, 10)

    def listener_callback_fsd(self,data):
        Angle = int(data.yaw)
        longitude = int(data.longitude)+2000
        latitude = int(data.latitude)+2700
        self.get_logger().debug("x={} y={} Angle={}".format(longitude,latitude,Angle))


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("vr_test")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()