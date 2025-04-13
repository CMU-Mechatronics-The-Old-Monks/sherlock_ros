import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    world_file = '/usr/share/gazebo-11/worlds/empty.world'

    return LaunchDescription([
        # Launch Gazebo server (no ROS plugins)
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_file],
            output='screen'
        ),

        # Launch Gazebo GUI
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # Launch robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
            ]),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Spawn the robot from the topic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
            output='screen'
        )
    ])
