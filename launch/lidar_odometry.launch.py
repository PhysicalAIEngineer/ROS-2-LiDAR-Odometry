from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "simple_lidar_odometry"
    package_share = Path(get_package_share_directory(package_name))
    rviz_config = package_share / "rviz" / "lidar.rviz"
    params_file = package_share / "config" / "odometry.yaml"

    lidar_odometry_node = Node(
        package=package_name,
        executable="lidar_odometry_node",
        name="lidar_odometry_node",
        output="screen",
        parameters=[str(params_file)],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(rviz_config)],
        output="screen",
    )

    return LaunchDescription([lidar_odometry_node, rviz_node])
