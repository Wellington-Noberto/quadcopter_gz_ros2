import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'drone_bringup'
    pkg_my_robot = FindPackageShare(package_name).find(package_name)
    model_path = os.path.join(pkg_my_robot, 'models', 'x500', 'model.sdf')


    entity_name_arg = DeclareLaunchArgument(
        'entity_name',
        default_value='spawned_model',
        description='Name of the spawned entity in Gazebo.'
    )
    world_file = 'default.sdf'
    world_config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'worlds',
        world_file
    ])

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 ', world_config_file])],
    )

    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', model_path,
            '-x', '1.0',
            '-y', '2.0',
            '-z', '0.5'
        ],
        output='screen'
    )


    return LaunchDescription([
        entity_name_arg,
        gazebo,
        gazebo_spawn_robot,
    ])
