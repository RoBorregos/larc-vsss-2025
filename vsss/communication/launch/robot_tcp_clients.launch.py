from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_name = "communication"
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    robots_params = os.path.join(pkg_share,'config' ,'robots_params.yaml')

    return LaunchDescription([
        Node(
           package= pkg_name,
           executable='robot_tcp_client_node.py',
           name='robot1_tcp_client',
           output='screen',
           parameters=[robots_params]
        ),
        #Node(
        #    package= pkg_name,
        #    executable='robot_tcp_client_node.py',
        #    name='robot2_tcp_client',
        #    output='screen',
        #    parameters=[robots_params]
        #),
        ##ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/robot1/cmd_vel
    ])
