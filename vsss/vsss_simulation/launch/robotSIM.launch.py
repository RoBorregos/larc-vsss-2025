#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, TextSubstitution, PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    
    robot_name = DeclareLaunchArgument(
        "robot_name", default_value="robot1", description="Robot Name"
    )
    robot_StartPosition = DeclareLaunchArgument(
        "robot_starting_pos", default_value="0.0", description="Posicion Inicial X"
    )
    robot_number = DeclareLaunchArgument(
        "robot_number", default_value="1", description="Robot Number"
    )
    pkg_name = "vsss_simulation"
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    robot_xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(["xacro ", robot_xacro_file, " robot_name:=", LaunchConfiguration("robot_name")])

    return LaunchDescription(
        [
            robot_name,   
            robot_number,
            robot_StartPosition,
            GroupAction([
                PushRosNamespace(LaunchConfiguration("robot_name")),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'robot_description': ParameterValue(robot_description,
                         value_type=str)
                    }]
                ),
                TimerAction(
                    period=3.0,  # Stagger spawn times
                    actions=[
                        Node(
                            package="gazebo_ros",
                            executable="spawn_entity.py",
                            arguments=[
                                "-topic", PathJoinSubstitution( [LaunchConfiguration("robot_name"), TextSubstitution(text ="robot_description")]),
                                "-entity", LaunchConfiguration("robot_name") ,
                                "-x", LaunchConfiguration("robot_starting_pos"),
                                "-y", "0.0",
                                "-z", "1"
                            ],
                            output="screen"
                        )
                        
                    ]
                ),
                     # Load joint_state_broadcaster
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager',  'controller_manager'],
                    output='screen'
                ),
                        # Load diff_drive_controller
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_cont', '--controller-manager',  'controller_manager'],
                    output='screen'
                ),

                Node(
                package="odom_to_tf_ros2",
                executable="odom_to_tf",
                name="odom_to_tf2",
                output="screen",
                parameters=[
                    {
                        "odom_topic": PathJoinSubstitution( [ TextSubstitution(text="odom")]),
                        "frame_id": "world",
                        "child_frame_id": [LaunchConfiguration("robot_name") ,"_base_link"],
                        "inverse_tf": False,
                        "use_sim_time": True,
                        "use_original_timestamp": False,
                    }
                ],
                remappings=[],
            )

            ])
        ]
    )