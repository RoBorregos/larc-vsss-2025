#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


# File: /ros/vsss_ws/src/vsss_simulation/launch/FieldPositions.launch.py
# Minimal ROS 2 Python launch file to start a field positions publisher node.
# Declares launch args: use_sim_time, field_config, and optional rviz start.




def generate_launch_description():
        #Spawn the goal transform
    goal = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output = "screen",
                name='goal_tf',
                arguments=[
                    "0.75",
                    "0.0",
                    "0.0",     
                    "0",
                    "0",
                    "0",
                    "world",
                    "goal_pos"
                ]
            )
    lower_end = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output="screen",
                name='own_lower_end_goal',
                arguments=[
                    "-0.75",
                    "0.35",
                    "0.0",
                    "0",
                    "0",
                    "0",
                    "world",
                    "own_lower_end_goal"
                ]
            )
    upper_end = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output="screen",
                name='own_upper_end_goal',
                arguments=[
                    "-0.75",
                    "-0.35",
                    "0.0",
                    "0",
                    "0",
                    "0",
                    "world",
                    "own_upper_end_goal"
                ]
            )
    world = Node(
        package='tf2_ros',
                executable='static_transform_publisher',
                output="screen",
                name='world',
                arguments=[
                    "0.75",
                    "-0.65",
                    "0.0",
                    "0",
                    "0",
                    "0",
                    "upper_left_corner",
                    "world"
                ]
    )

    actions = [
        goal,
        lower_end,
        upper_end,
        world
    ]


    return LaunchDescription(actions)