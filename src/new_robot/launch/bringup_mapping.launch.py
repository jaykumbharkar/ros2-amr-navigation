from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    new_robot_pkg = get_package_share_directory('new_robot')
    fourwd_pkg = get_package_share_directory('fourwd_controller')

    # ---------------- Gazebo ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                new_robot_pkg,
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # ---------------- EKF ----------------
    ekf = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        fourwd_pkg,
                        'launch',
                        'ekf.launch.py'
                    )
                )
            )
        ]
    )

    # ---------------- SLAM Toolbox ----------------
    slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file':
                    os.path.join(
                        new_robot_pkg,
                        'config',
                        'mapper_params_online_async.yaml'
                    )
                }.items()
            )
        ]
    )

    # ---------------- RViz ----------------
    rviz = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        ekf,
        slam,
        rviz
    ])
