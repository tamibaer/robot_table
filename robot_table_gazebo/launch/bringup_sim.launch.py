from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
import os


def generate_launch_description():

    pkg_gazebo = FindPackageShare("robot_table_gazebo").find("robot_table_gazebo")
    pkg_moveit = FindPackageShare("robot_table_moveit_config").find("robot_table_moveit_config")

    sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "sim_control.launch.py")
        )
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "move_group_sim.launch.py")
        )
    )

    return LaunchDescription([
        sim_control,
        moveit
    ])