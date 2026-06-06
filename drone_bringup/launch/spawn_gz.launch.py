import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_name = 'drone_bringup'    
    world_file = 'worlds/default.sdf'
    roz_gz_bridge_file = 'config/ros_gz_bridge.yaml'
    quadcopter_model_file = 'models/x500/model.sdf'
    
 
    world_config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        world_file
    ])

    gazebo_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 ', world_config_file])],
    )

    # Path to ros_gz_bridge config YAML (placed in package's config folder)
    bridge_config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        roz_gz_bridge_file
    ])

    # Launch ros_gz_bridge parameter_bridge with the config file
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config_file}],
        output='screen'
    )

    quad_model_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        quadcopter_model_file
    ])

    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', quad_model_path,
            '-x', '1.0',
            '-y', '2.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        gazebo_spawn_robot,
        ros_gz_bridge,
    ])
