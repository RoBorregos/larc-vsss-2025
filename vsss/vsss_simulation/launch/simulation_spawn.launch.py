#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_name = "vsss_simulation"
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    gazebo_model_path = os.path.join(pkg_share, "urdf", "PistaVSSS_URDF.urdf")
    ball_file = os.path.join(pkg_share, 'urdf', 'ball.urdf')
    camera_file = os.path.join(pkg_share, 'urdf', 'camera.urdf')
    config_file_path = os.path.join(pkg_share, 'config', 'ball_odom2_tf.yaml')


    robot_count = 2  # Number of robots
    robot_spawns = []
    for i in range(robot_count):
        robotName = f"robot{i+1}"
        position =  i *1.0
        if(i == 0):
            controlNode = Node(
                package=pkg_name,
                namespace= robotName,
                executable="RobotController",
                parameters=[{"number":(i+1)}],
                output="screen"
            )
        else:
            controlNode = Node(
                package=pkg_name,
                namespace=robotName,
                executable="DefaultController.py",
                parameters=[ {"robot_namespace": robotName}]
            )


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
                }.items()
            ),
            controlNode
            
        ])
        robot_spawns.append(
            robotNodes
        )


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
            ])
        ),

        # Spawn field model (from file)
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-file", gazebo_model_path,
                "-entity", "Field",
                        "-x", "0.0",  # X position
                        "-y", "0.0",  # Y position
                        "-z", "60.0",  # Z position
                        "-R", "0",    # Roll
                        "-P", "0",    # Pitch
                        "-Y", "0"     # Yaw
    ],
            output="screen"
        ),
       
        
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
                        "-x", "-2.0",  # X position
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
        #Spawn the goal transform
        TimerAction(
            period=2.0,
            actions=[Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    output = "screen",
                    name='goal_tf',
                    arguments=[
                        "-3.0",
                        "0.0",
                        "0.0",     
                        "0",
                        "0",
                        "0",
                        "world",
                        "goal_pos"
                    ]
                )]
            ),
        

        #Robots Nodes
        *robot_spawns
        

    ])
