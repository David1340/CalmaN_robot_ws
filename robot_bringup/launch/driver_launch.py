# driver_launch.py
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, LifecycleNode
import os

def generate_launch_description():
        
    description_launch = os.path.join(
        get_package_share_directory('calman_description'),
        'launch',
        'rsp.launch.py'
    )

    lidar_launch = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )

        
    return LaunchDescription([
        #Node(package='robot_lidar', 
        #    executable='lidarX4_node',
        #    name='robot_lidar',
        #    output='screen'),
        Node(package='robot_camera',
            executable='camera_node', 
            name='robot_camera', 
            output='screen'),
        Node(package='stm32_bridge',
            executable='stm32_bridge',
            name='stm32_bridge_node',
            output='screen'),
        #Node(package='robot_bringup', 
        #    executable='encoder_node',
        #    name='encoder_node_odom',
        #    output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)
        ),

    ])
