#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare variables (equivalent to <let> in XML)
    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_bot1_description'),
        'urdf',
        'my_bot1.xacro'
    ])
    
    gazebo_config_path = PathJoinSubstitution([
        FindPackageShare('my_bot1_description'),
        'config',
        'my_bot1_gz_bridge.yaml'
    ])
    
    gazebo_world = PathJoinSubstitution([
        FindPackageShare('navigation_bringup'),
        'worlds',
        'hospital.sdf'
    ])
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )
    
    # ROS-Gazebo Bridge Node
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gazebo_config_path
        }]
    )
    
    # Include Gazebo Sim Launch File
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', gazebo_world]
        }.items()
    )
    
    # Gazebo Create Node (spawn robot)
    gazebo_create_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'hospital_world',
            '-topic', 'robot_description'
        ]
    )
    
    # Lidar Scan Node
    #lidar_scan_node = Node(
    #    package='my_bot1_description',
    #    executable='lidar_scan_node.py'
    #)
    
    # Command Navigation to Command Node
    cmd_nav_to_cmd_node = Node(
        package='my_bot1_description',
        executable='cmd_nav_to_cmd.py'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        ros_gz_bridge_node,
        gazebo_sim,
        gazebo_create_node,
        #lidar_scan_node,
        cmd_nav_to_cmd_node
    ])