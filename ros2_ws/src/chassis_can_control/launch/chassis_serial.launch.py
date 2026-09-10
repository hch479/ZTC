from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    package_directory = get_package_share_directory("chassis_can_control")
    parameter_file = os.path.join(package_directory, "config", "chassis_serial.yaml")
    return LaunchDescription(
        [
            Node(
                package="chassis_can_control",
                executable="chassis_can_node",
                name="chassis_can_node",
                output="screen",
                parameters=[parameter_file],
            )
        ]
    )
