from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类def 

def generate_launch_description():
    run_nodes = []
    run_nodes.append(Node(package='car_5_ori',executable='com'))
    run_nodes.append(Node(package='car_3_determine_location',executable='magnetic'))
    run_nodes.append(Node(package='car_5_ori',executable='car_ori'))
    run_nodes.append(Node(package='car_7_path_planner',executable='global_path_planning'))
    run_nodes.append(Node(package='car_7_path_planner',executable='magnetic_local_path_planning'))
    run_nodes.append(Node(package='car_8_controller',executable='pid'))
    run_nodes.append(Node(package='car_6_fusion',executable='fusion'))
    run_nodes.append(Node(package='car_4_communication',executable='net_work'))

    node1 = Node(package='car_5_ori',executable='com')
    node2 = Node(package='car_3_determine_location',executable='magnetic')
    node3 = Node(package='car_3_determine_location',executable='rfid')
    # node4 = Node(package='car_3_determine_location',executable='imu')
    # node5 = Node(package='car_2_sonic_obstacle',executable='sonic_obstacle')
    node6 = Node(package='car_5_ori',executable='car_ori')
    node7 = Node(package='car_7_path_planner',executable='global_path_planning')
    node8 = Node(package='car_7_path_planner',executable='magnetic_local_path_planning')
    node9 = Node(package='car_8_controller',executable='pid')
    node10 = Node(package='car_6_fusion',executable='fusion')
    node11 = Node(package='car_4_communication',executable='net_work')

    # node12 = Node(package='car_4_communication',executable='point_data_bag')


    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '1', '0', '0', '0', 'world', 'mystaticturtle'])

    launch_description = LaunchDescription(run_nodes)
    return launch_description

