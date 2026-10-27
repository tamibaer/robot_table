from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from robot_table_control.controller_config import write_controllers_file

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",  # put your robot's IP address here
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_admittance",
            default_value="false",
            description="Arm-Controller hinter den F/T-basierten admittance_controller haengen?",
        )
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "controller_manager"]
    )

    robotiq_activation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_activation_controller", "-c", "controller_manager"]
    )

    def launch_robot(context):
        use_admittance = LaunchConfiguration("use_admittance").perform(context) == "true"
        overlays = ["ros2_controllers.yaml"]
        if use_admittance:
            overlays.append("admittance_controllers.yaml")
        controllers_file = write_controllers_file(overlays)
        print(f"Controller-Konfiguration: {controllers_file}")

        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur_robot_driver"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "tf_prefix": [LaunchConfiguration("ur_type"), "_"],
                    "rviz_config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("robot_table_description"),
                            "rviz",
                            "urdf.rviz",
                        ]
                    ),
                    "description_launchfile": PathJoinSubstitution(
                        [
                            FindPackageShare("robot_table_control"),
                            "launch",
                            "rsp.launch.py",
                        ]
                    ),
                    "controllers_file": controllers_file,
                    "activate_joint_controller": "false" if use_admittance else "true",
                }.items(),
            ),
            robotiq_gripper_controller_spawner, robotiq_activation_controller_spawner
        ]
        if use_admittance:
            actions += [
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["admittance_controller", "-c", "controller_manager", "--inactive"],
                ),
                Node(
                    package="robot_table_control",
                    executable="activate_controllers",
                    arguments=["admittance_controller", "scaled_joint_trajectory_controller"],
                    output="screen",
                ),
            ]
        return actions

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_robot)])
