from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_name = "communication"
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    param_dir = os.path.join(pkg_share, 'config')

    robot1_params = os.path.join(param_dir, 'robot1_params.yaml')
    robot2_params = os.path.join(param_dir, 'robot2_params.yaml')
    print(param_dir)
    return LaunchDescription([
        Node(
            package='communication',
            executable='robot_tcp_client_node.py',
            name='robot1_tcp_client',
            output='screen',
            parameters=[robot1_params]
        ),
        ##ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/robot1/cmd_vel
    ])
