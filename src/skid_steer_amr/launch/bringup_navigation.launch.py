"""Navigate on the saved map: Gazebo + EKF + map_server/AMCL + Nav2 + RViz.

Set goals in RViz with "2D Goal Pose".

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

    def include(launch_file, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', launch_file)
            ),
            **kwargs
        )

    # Started in sequence so each stage finds the TF/topics it needs
    gazebo = include('gazebo.launch.py', launch_arguments={'gui': gui}.items())
    ekf = TimerAction(period=3.0, actions=[include('ekf.launch.py')])
    localization = TimerAction(period=8.0, actions=[include('localization.launch.py')])
    navigation = TimerAction(period=13.0, actions=[include('navigation.launch.py')])

    rviz = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
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
        localization,
        navigation,
        rviz
    ])
