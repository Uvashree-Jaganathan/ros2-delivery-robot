from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'resolution': 0.05
            }]
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                )
            ]
        )
    ])
