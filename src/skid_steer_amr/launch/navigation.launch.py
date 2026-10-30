"""Nav2 navigation servers (planner, controller, behaviors, BT navigator, ...)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('skid_steer_amr')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
        }.items()
    )

    # Camera depth image -> LaserScan. Nav2 uses it to CLEAR cells the camera
    # can see through, so people who walk away do not leave ghost obstacles.
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/camera/scan'),
        ],
        parameters=[{
            'use_sim_time': True,
            'scan_height': 20,
            'range_min': 0.1,
            'range_max': 6.0,
            'output_frame': 'camera_link',
        }]
    )

    return LaunchDescription([
        nav2,
        depth_to_scan
    ])
