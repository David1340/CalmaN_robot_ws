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

    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py'
    )
        
    return LaunchDescription([
        Node(package='stm32_bridge',
            executable='stm32_bridge',
            name='stm32_bridge_node',
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch)
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0'
        }.items()
    )
    ])
