import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    my_bot_dir = get_package_share_directory('my_bot')

    return LaunchDescription([
        # 🔹 Gazebo simulation with robot spawn
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_bot_dir, 'launch', 'launch_sim.launch.py')
            )
        ),

        # 🔹 SLAM Toolbox (async) with mapper params
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_bot_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': os.path.join(my_bot_dir, 'config', 'mapper_params_online_async.yaml'),
            }.items()
        ),

        # 🔹 Nav2 Global Planner with nav2_params and map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_bot_dir, 'launch', 'global_planner_only.launch.py')
            ),
            launch_arguments={
                'map': os.path.join(my_bot_dir, 'config', 'map1.yaml'),
                'params_file': os.path.join(my_bot_dir, 'config', 'nav2_params.yaml'),
            }.items()
        ),

        # 🔹 Mecanum Nodes
        Node(
            package='mecanum_nodes',
            executable='planner',
            name='planner_node',
            output='screen'
        ),
        Node(
            package='mecanum_nodes',
            executable='controller',
            name='fbController',
            output='screen'
        ),
        Node(
            package='mecanum_nodes',
            executable='serial_sub',
            name='serial_subscriber',
            output='screen'
        ),
        Node(
            package='mecanum_nodes',
            executable='stepper_pub',
            name='stepper_publisher',
            output='screen'
        ),

        # 🔹 My Bot Nodes
        Node(
            package='my_bot',
            executable='filtered_path_publisher_node',
            name='filtered_path_publisher',
            output='screen'
        ),
        Node(
            package='my_bot',
            executable='goal_publisher_node',
            name='goal_publisher',
            output='screen'
        ),
    ])
