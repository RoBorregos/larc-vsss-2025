#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, TextSubstitution, PathJoinSubstitution, LaunchConfiguration



def generate_launch_description():
    
    robot_count = 1
    robot_controllers = []
    pkg_name = "vsss_simulation"

    strategiest_side = DeclareLaunchArgument(
        "team_side",
        default_value="false",
        description="Team side flag (true/false)"
    )
    # use this when passing into node parameters to get a real bool:

    Strat = Node(
            package=pkg_name,
            executable = "Strategist",
            name = "strategist",
            output = "screen",
            parameters=[{"Robot_count":(robot_count) , "Robot_side": LaunchConfiguration("team_side")}],
        )

    for i in range(robot_count):
        robotName = "robot"+str(i+1)
        robot_controllers.append(
            Node(
                package=pkg_name,
                namespace= robotName,
                executable="RobotController",
                parameters=[{"number":(i+1)}],
                output="screen"
            )
        )
     ##Launch static transforms
    StaticPoints = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.path.dirname(__file__),
                    'FieldPositions.launch.py'
                )
            )
        )

    
    return LaunchDescription([strategiest_side, StaticPoints, Strat, *robot_controllers])
    