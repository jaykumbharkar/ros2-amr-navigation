"""Start Gazebo Classic with the hospital world and spawn the robot.

Arguments:
  gui:=true|false   start the Gazebo client window (default true)
  world:=<file>     world file in this package's worlds/ folder (default hospital.world)
  x:= y:= yaw:=     robot spawn pose (default: main lobby, facing the ward doors)
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('skid_steer_amr')

    xacro_file = os.path.join(pkg_share, 'urdf', 'skid_steer_amr.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    world = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])

    # Raises /clock from the default 10 Hz to 100 Hz
    gazebo_params = os.path.join(pkg_share, 'config', 'gazebo_params.yaml')

    # Lets Gazebo resolve package://skid_steer_amr/meshes/... :
    # it looks for a folder named skid_steer_amr inside GAZEBO_MODEL_PATH.
    model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.dirname(pkg_share)
    )

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            # ROS args must be wrapped in --ros-args ... -- for gzserver
            '--ros-args', '--params-file', gazebo_params, '--'
        ],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'skid_steer_amr',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.15',
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo client window'
        ),
        DeclareLaunchArgument(
            'world', default_value='hospital.world',
            description='World file in the worlds/ folder'
        ),
        # Default spawn: main lobby, facing the ward doors.
        # AMCL's initial_pose in nav2_params.yaml must match.
        DeclareLaunchArgument('x', default_value='-8.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
        model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn
    ])
