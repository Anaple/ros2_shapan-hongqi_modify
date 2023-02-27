import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    node1 = launch_ros.actions.Node(package='car_2_sonic_obstacle',node_executable='sonic_obstacle',output='screen',)
    node2 = launch_ros.actions.Node(package='car_3_determine_location',node_executable='magnetic',output='screen')
    node3 = launch_ros.actions.Node(package='car_3_determine_location',node_executable='imu',output='screen')
    node4 = launch_ros.actions.Node(package='car_3_determine_location',node_executable='gps',output='screen')
    node5 = launch_ros.actions.Node(package='car_4_communication',node_executable='server_work',output='screen')
    node6 = launch_ros.actions.Node(package='car_4_communication',node_executable='net_work',output='screen')
    node7 = launch_ros.actions.Node(package='car_5_ori',node_executable='car_ori',output='screen')
    node8 = launch_ros.actions.Node(package='car_5_ori',node_executable='com',output='screen')
    node9 = launch_ros.actions.Node(package='car_6_fusion',node_executable='fusion',output='screen')
    node10 = launch_ros.actions.Node(package='car_7_path_planner',node_executable='global_path_planning',output='screen')
    node11 = launch_ros.actions.Node(package='car_7_path_planner',node_executable='local_path_planning',output='screen')
    node12 = launch_ros.actions.Node(package='car_8_controller',node_executable='pid',output='screen')
    launch_description = launch.LaunchDescription([node8,node2])
    return launch_description

