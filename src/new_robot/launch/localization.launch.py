from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("new_robot")

    map_yaml_path = os.path.join(
        pkg_share,
        "maps",
        "hospital_map.yaml"
    )

    nav2_params = os.path.join(
        pkg_share,
        "config",
        "nav2_params.yaml"
    )

    # ---------------- Map Server ----------------
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "yaml_filename": map_yaml_path
        }]
    )

    # ---------------- AMCL ----------------
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params]
    )

    # ---------------- Lifecycle Manager ----------------
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": [
                "map_server",
                "amcl"
            ]
        }]
    )

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager
    ])
