# driver_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_lidar', executable='lidarX4_node', name='robot_lidar', output='screen'),
        #Node(package='ydlidar',
        #    executable='ydlidar_node',
        #    name='robot_lidar',
        #    parameters=['../../ydlidar_ros2/params/ydlidar.yaml'],
        #    output='screen'),
        Node(package='robot_camera',
            executable='camera_node', 
            name='robot_camera', 
            output='screen'),
        Node(package='stm32_bridge',
            executable='stm32_bridge',
            name='stm32_bridge_node',
            output='screen'),
    ])
