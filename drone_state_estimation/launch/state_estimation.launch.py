

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'drone_state_estimation'    
    state_estimation_node_name = 'state_estimation_node'    
    state_estimation_config_filename = 'config/state_estimation.yaml'
    ekf_config_filename = 'config/ekf.yaml'
    
 
    config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        state_estimation_config_filename
    ])

    state_estimation_node = Node(
            package=pkg_name,
            executable=state_estimation_node_name,
            name=state_estimation_node_name,
            output='screen',
            parameters=[config_file],
    )

    ekf_config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        ekf_config_filename
    ])

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file],
    )



    return LaunchDescription([
        state_estimation_node,
        ekf_node,
    ])
