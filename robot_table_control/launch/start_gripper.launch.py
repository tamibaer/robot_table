import launch
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
import launch_ros
import os

def generate_launch_description():

    # Paket mit den ros2_control-Tags + Gripper
    control_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="robot_table_control"
    ).find("robot_table_control")

    # Controller-XACRO (referenziert die Description-Macros intern)
    default_model_path = os.path.join(
        control_pkg_share, "urdf", "robot_table_controlled.urdf.xacro"
    )

    # Optional: eigenes RViz
    default_rviz_config_path = os.path.join(
        control_pkg_share, "rviz", "view_urdf.rviz"
    )

    args = [
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="URDF/XACRO file with controller tags"
        ),
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="RViz config file"
        ),
        launch.actions.DeclareLaunchArgument(
            name="launch_rviz",
            default_value="false",
            description="Launch RViz?"
        ),
        launch.actions.DeclareLaunchArgument(
            name="com_port",
            default_value="/tmp/ttyUR",
            description="Port for Robotiq Gripper"
        ),
        launch.actions.DeclareLaunchArgument(
            name="use_mock_hardware",
            default_value="false",
            description="Use fake hardware for testing"
        ),
        launch.actions.DeclareLaunchArgument(
            name="ur_type",
            default_value="ur5e",
            description="UR robot type"
        ),
    ]

    # xacro Command
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        LaunchConfiguration("model"),
        " ",
        "com_port:=", LaunchConfiguration("com_port"),
        " ",
        "use_mock_hardware:=", LaunchConfiguration("use_mock_hardware"),
        " ",
        "ur_type:=", LaunchConfiguration("ur_type"),
    ])

    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # Update-Rate / Controller YAML falls vorhanden
    update_rate_config_file = PathJoinSubstitution([
        control_pkg_share,
        "config",
        "update_rate.yaml"
    ])

    initial_joint_controllers = PathJoinSubstitution([
        control_pkg_share,
        "config",
        "ros2_controllers.yaml"
    ])

    # ros2_control node
    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            update_rate_config_file,
            initial_joint_controllers
        ],
        namespace="gripper"
    )

    # State publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
        namespace="gripper"
    )

    # RViz
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        condition=IfCondition(LaunchConfiguration("launch_rviz"))
    )

    # Spawner für Controller
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/gripper/controller_manager"]
    )

    robotiq_gripper_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/gripper/controller_manager"]
    )

    robotiq_activation_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_activation_controller", "-c", "/gripper/controller_manager"]
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robotiq_gripper_controller_spawner,
        robotiq_activation_controller_spawner,
        rviz_node
    ]

    return launch.LaunchDescription(args + nodes)
