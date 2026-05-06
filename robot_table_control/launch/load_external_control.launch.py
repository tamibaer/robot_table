from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='robot_table_control',
            executable='dashboard_client',
            name='dashboard_client',
            output='screen'
        )
    ])