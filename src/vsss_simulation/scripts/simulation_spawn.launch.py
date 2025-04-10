#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_name = "vsss_simulation"
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    gazebo_model_path = os.path.join(pkg_share, "worlds", "vsss_world")

    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    ball_file = os.path.join(pkg_share, 'urdf', 'ball.urdf')
    camera_file = os.path.join(pkg_share, 'urdf', 'camera.urdf')
    robot_description = Command(['xacro ', xacro_file])

    controller_config = os.path.join(pkg_share, 'config', 'diff_drive_controllers.yaml')

    world_file = os.path.join(pkg_share, 'worlds', 'nothing.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Launch Gazebo with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description
            }]
        ),

        # Spawn robot in Gazebo (with delay to avoid race condition)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-topic", "/robot_description",
                        "-entity", "vsss_robot",
                        "-x", "0.0",  # X position
                        "-y", "0.0",  # Y position
                        "-z", "0.2",  # Z position
                        "-R", "0",    # Roll
                        "-P", "0",    # Pitch
                        "-Y", "0"     # Yaw
                    ],
                    output="screen"
                )
            ]
        ),

        # Spawn wall model (from file)
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-file", os.path.join(gazebo_model_path, "model.sdf"),
                "-entity", "wall_model"
            ],
            output="screen"
        ),
        
        # Spawn robot in Gazebo (with delay to avoid race condition)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-file", ball_file,
                        "-entity", "ball",
                        "-x", "2.0",  # X position
                        "-y", "0.0",  # Y position
                        "-z", "0.5",  # Z position
                        "-R", "0",    # Roll
                        "-P", "0",    # Pitch
                        "-Y", "0"     # Yaw
                    ],
                    output="screen"
                ),
            ]
        ),

        # Spawn camera model (from file)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-file", camera_file,
                        "-entity", "camera_model",
                    ],
                    output="screen"
                ),
            ]
        ),


        # Load joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Load diff_drive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
            output='screen'
        ),
    ])
