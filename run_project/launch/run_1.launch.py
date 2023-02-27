import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   vrpn_client_ros = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('vrpn_client_ros'), 'launch'),
         '/sample.launch.py'])
      )
   run_project = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('run_project'), 'launch'),
         '/run_2.launch.py'])
      )
   return LaunchDescription([
      vrpn_client_ros,
      run_project
   ])
