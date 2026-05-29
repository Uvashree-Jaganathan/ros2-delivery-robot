from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_params = PathJoinSubstitution([
        FindPackageShare('delivery_bot_navigation'),
        'config',
        'nav2_params.yaml'
    ])

    map_file = PathJoinSubstitution([
        FindPackageShare('delivery_bot_navigation'),
        'maps',
        'indoor_delivery_map.yaml'
    ])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_file,
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
