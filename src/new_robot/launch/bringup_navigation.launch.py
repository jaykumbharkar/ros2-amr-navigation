from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("new_robot")
    fourwd_pkg = get_package_share_directory("fourwd_controller")

    # ---------------- Gazebo ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share,
                "launch",
                "gazebo.launch.py"
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
                        "launch",
                        "ekf.launch.py"
                    )
                )
            )
        ]
    )

    # ---------------- Localization ----------------
    localization = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_share,
                        "launch",
                        "localization.launch.py"
                    )
                )
            )
        ]
    )

    # ---------------- Navigation ----------------
    navigation = TimerAction(
        period=13.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_share,
                        "launch",
                        "navigation.launch.py"
                    )
                )
            )
        ]
    )

    # ---------------- RViz ----------------
    rviz = TimerAction(
        period=16.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                parameters=[
                    {"use_sim_time": True}
                ]
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        ekf,
        localization,
        navigation,
        rviz
    ])
