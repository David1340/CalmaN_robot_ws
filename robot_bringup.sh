  GNU nano 6.2                                        robot_bringup.sh
#!/bin/bash
# ==== ROS2 Bringup Auto-Start Script ====

# call ROS 2 environment
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
source calman_ws/install/local_setup.bash


########## ground robot #########
export ROS_DOMAIN_ID=1
ls /dev/ttyUSB*
call bringup
ros2 launch robot_bringup driver_launch.py
