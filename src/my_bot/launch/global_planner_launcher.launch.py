from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'map': map_yaml_file
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map', description='path to yaml map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_dir, 'config', 'global_planner.yaml'),
            description='Full path to global planner parameters file to use for all launched nodes'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings + [('plan', 'com_trajector')]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['planner_server']
            }]
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='simple_goal_sender',
                    name='goal_publisher',
                    output='screen'
                )
            ]            
        )
    ])
        