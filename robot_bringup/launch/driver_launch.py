# driver_launch.py
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
        
    description_launch = os.path.join(
        get_package_share_directory('calman_description'),
        'launch',
        'rsp.launch.py'
    )
        
    return LaunchDescription([
        Node(package='robot_lidar', 
            executable='lidarX4_node',
            name='robot_lidar',
            output='screen'),
        #Node(package='ydlidar',
            #executable='ydlidar_node',
            #name='robot_lidar',
            #parameters=['../../ydlidar_ros2/params/ydlidar.yaml'],
            #output='screen'),
        Node(package='robot_camera',
            executable='camera_node', 
            name='robot_camera', 
            output='screen'),
        Node(package='stm32_bridge',
            executable='stm32_bridge',
            name='stm32_bridge_node',
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch)
        ),
    ])
