from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    device_name = LaunchConfiguration("device_name")

    declared_arguments = [
        DeclareLaunchArgument("robot_ip", default_value="192.168.56.101"),
        DeclareLaunchArgument("device_name", default_value="/tmp/ttyUR"),
    ]

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="ur_robot_driver",
                executable="tool_communication.py",
                output="screen",
                parameters=[
                    {"robot_ip": robot_ip, "device_name": device_name}
                ],
            )
        ]
    )
