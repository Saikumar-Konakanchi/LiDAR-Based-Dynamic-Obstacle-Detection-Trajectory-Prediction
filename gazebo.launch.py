import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('Lidar_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'Lidar_gazebo.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'tracking_world.world')

    return LaunchDescription([
        # Start Gazebo with the tracking world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_file,
            ],
            output='screen',
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(urdf_file).read(),
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # Spawn the rover after Gazebo has had time to start
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                        '-file', urdf_file,
                        '-entity', 'lidar_rover',
                        '-x', '-3.0',
                        '-y', '0.0',
                        '-z', '0.1',
                    ],
                    output='screen',
                )
            ],
        ),
    ])
