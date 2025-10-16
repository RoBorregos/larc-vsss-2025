#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, TextSubstitution, LaunchConfiguration
import random
 
def generate_launch_description():
    pkg_name = "vsss_simulation"
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    world_file = os.path.join(pkg_share, "worlds", "vsss_complete.world")
    ball_file = os.path.join(pkg_share, 'urdf', 'ball.urdf')
    camera_file = os.path.join(pkg_share, 'urdf', 'camera.urdf')
    config_file_path = os.path.join(pkg_share, 'config', 'ball_odom2_tf.yaml')

    robot_count = 2
    robot_spawns = []
    team_colors = ['Blue', 'Yellow']  # Colors for the teams
    robot_colors = [ 'Green', 'Turquoise', 'Purple']  # Colors for the robots
    random.seed()


    strategiest_side = DeclareLaunchArgument(
        "team_side",
        default_value="false",
        description="Team side flag (true/false)"
    )
    
    for i in range(robot_count):
        robotName = f"robot{i+1}"
        position = i * 0.2
        #alternating between the available team identifier colors
        dominant_color = team_colors[i % len(team_colors)]

        #--------------------------Assign color to small plate's system--------------------
        # #assign small plates colors 
        # color_index_1 = i % len(robot_colors)
        # color_index_2 = (i + 1) % len(robot_colors)

        # #ensure different colors in small plates
        # if color_index_1 == color_index_2:
        #     color_index_2 = (color_index_2 + 1) % len(robot_colors)

        # small_plate1_color = robot_colors[color_index_1]
        # small_plate2_color = robot_colors[color_index_2]

        if(i < 2):
            controlNode = Node(
                package=pkg_name,
                namespace= robotName,
                executable="RobotController",
                parameters=[
                {"number":(i+1) ,
                 "team_side":LaunchConfiguration("team_side")}],
                output="screen"
            )
        else:
            controlNode = Node(
                package=pkg_name,
                namespace=robotName,
                executable="DefaultController.py",
                parameters=[ {"robot_namespace": robotName}]
        )

        small_plate1_color = random.choice(robot_colors)
        small_plate2_color = random.choice(robot_colors)

        while (small_plate1_color == small_plate2_color):
            small_plate2_color = random.choice(robot_colors)

        robotNodes = TimerAction(
            period=4.0* i + 6.0,
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.dirname(__file__),
                        'robotSIM.launch.py'
                    )
                ),
                launch_arguments= {
                    'robot_name' : robotName,
                    'robot_starting_pos' : str(position),
                    'robot_number' : str(i+1),
                    'dominant_plate_color' : dominant_color,
                    'small_plate_1' : small_plate1_color,
                    'small_plate_2' : small_plate2_color,
                }.items()
            ),
            controlNode
            
        ])
        robot_spawns.append(
            robotNodes
        )


    return LaunchDescription([
        strategiest_side,
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Set GAZEBO_MODEL_PATH to include desired models
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', 
                              os.path.join(pkg_share, "worlds", "vsss_worlds") + ":" + 
                              os.environ.get('GAZEBO_MODEL_PATH', '')),

        # Launch Gazebo with custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ]),
            launch_arguments={
                'world': world_file,
                'extra_gazebo_args': '--verbose'
            }.items()
        ),
        ##Launch static transforms
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.path.dirname(__file__),
                    'FieldPositions.launch.py'
                )
            )
        ),
        # No need for spawn entity because it is already in wordl.
       
        
        # Spawn ball in Gazebo (with delay to avoid race condition)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-file", ball_file,
                        "-entity", "ball",
                        "-x", "-0.2",  # X position
                        "-y", "0.0",  # Y position
                        "-z", "1.0",  # Z position
                        "-R", "0",    # Roll
                        "-P", "0",    # Pitch
                        "-Y", "0"     # Yaw
                    ],
                    output="screen"
                )
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

        #Spawn Node to see the ball
        Node(
                package="odom_to_tf_ros2",
                executable="odom_to_tf",
                name="odom_to_tf",
                output="screen",
                parameters=[config_file_path],
                remappings=[],
            ),

        #Spawn Node of Strategist
        Node(
            package=pkg_name,
            executable = "Strategist",
            name = "strategist",
            output = "screen",
            parameters=[{"Robot_count": robot_count, "Robot_side": LaunchConfiguration("team_side")}],
        ),

        Node(
            package = pkg_name,
            executable = "global_parameter_server_node",
            name = "global_parameter_server_node",
            output="screen"
        ),
        

        #Robots Nodes
        *robot_spawns
        

    ])
