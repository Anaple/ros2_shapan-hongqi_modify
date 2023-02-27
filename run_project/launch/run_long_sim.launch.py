from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类def 

def generate_launch_description():
    node1 = Node(package='car_2_sonic_obstacle',executable='sonic_obstacle')
    node2 = Node(package='car_3_determine_location',executable='magnetic')
    node3 = Node(package='car_4_communication',executable='server_work')
    node4 = Node(package='car_4_communication',executable='net_work')
    node5 = Node(package='car_3_determine_location',executable='simulation_gps')
    node6 = Node(package='car_5_ori',executable='car_control')
    node7 = Node(package='car_6_fusion',executable='fusion')
    node8 = Node(package='car_7_path_planner',executable='global_path_planning')
    node9 = Node(package='car_7_path_planner',executable='local_path_planning')
    node10 = Node(package='car_8_controller',executable='pid')
    print(666)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '1', '0', '0', '0', 'world', 'mystaticturtle'])

    launch_description = LaunchDescription([node5,node7])
    return launch_description

