from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类def 

def generate_launch_description():
    run_nodes = []
    run_nodes.append(Node(package='car_4_communication',executable='net_work'))
    




    # node12 = Node(package='car_4_communication',executable='point_data_bag')


    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '1', '0', '0', '0', 'world', 'mystaticturtle'])

    launch_description = LaunchDescription(run_nodes)
    return launch_description