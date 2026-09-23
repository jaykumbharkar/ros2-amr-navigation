import os
import xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('new_robot')

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'new_robot.urdf.xacro'
    )

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    robot_description = {
        "robot_description": doc.toxml()
    }
    world = os.path.join(
        pkg_path,
        'worlds',
        'hospital_simple.world'
    )
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
        output="screen",
    )

    spawn = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-topic', 'robot_description',
        '-entity', 'new_robot',

        '-x', '4.0',
        '-y', '0.0',
        '-z', '0.15',

        '-R', '0.0',
        '-P', '0.0',
        '-Y', '0.0'
    ],
    output='screen'
   )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn
    ])
