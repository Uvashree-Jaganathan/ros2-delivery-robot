from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('delivery_bot_gazebo'),
        'worlds',
        'indoor_delivery_world.sdf'
    ])

    robot_urdf = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'delivery_bot.urdf.xacro'
    ])

    robot_description = {
        'robot_description': Command(['xacro ', robot_urdf])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'delivery_bot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '1.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel_safe@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    safety_stop = Node(
        package='delivery_bot_perception_cpp',
        executable='safety_stop_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'stop_distance': 0.55,
            'slow_distance': 1.00,
            'scan_forward_angle': 0.90,
            'caution_speed_scale': 0.45,
            'recovery_turn_angular_speed': 0.45,
            'min_turn_command': 0.05,
            'cmd_timeout_ms': 500,
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        safety_stop
    ])
