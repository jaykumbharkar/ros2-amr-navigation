"""Build a map: Gazebo + EKF + SLAM Toolbox + RViz.

Drive the robot around (e.g. teleop_twist_keyboard), then save the map with
  ros2 run nav2_map_server map_saver_cli -f <name>

Arguments:
  gui:=true|false   start the Gazebo and RViz windows (default true)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('skid_steer_amr')
    gui = LaunchConfiguration('gui')

    # ---------------- Gazebo + robot ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gui': gui}.items()
    )

    # ---------------- EKF ----------------
    ekf = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'ekf.launch.py')
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
                    'slam_params_file': os.path.join(
                        pkg_share, 'config', 'mapper_params_online_async.yaml'
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
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'hospital_nav.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen',
                condition=IfCondition(gui)
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo and RViz windows'
        ),
        gazebo,
        ekf,
        slam,
        rviz
    ])
