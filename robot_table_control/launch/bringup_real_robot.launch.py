import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution 

from launch.event_handlers import OnTopicEvent

def generate_launch_description():
    pkg_robot_table_control = FindPackageShare('robot_table_control')

    tool_comm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_robot_table_control,
                'launch',
                'ur_tool_communication.launch.py'
            ])
        )
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_robot_table_control,
                'launch',
                'start_robot.launch.py'
            ])
        ),
        launch_arguments={
            'use_mock_hardware': 'false',
            'launch_rviz': 'false'
        }.items()
    )

    external_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_robot_table_control,
                'launch',
                'load_external_control.launch.py'
            ])
        )
    )
        
    trigger_external_control = RegisterEventHandler(
        OnTopicEvent(
            topic='/robot_ready',
            trigger_on_arrival=True,
            on_event=[external_control_launch]
        )
    )

    def check_ttyur(context, *args, **kwargs): 
        if os.path.exists("/tmp/ttyUR"):
            print("Device /tmp/ttyUR found, launching robot...")
            return [robot_launch]
        else:
            print("Device /tmp/ttyUR not found.")
            return []
   
    return LaunchDescription([
        tool_comm_launch,
        TimerAction(
            period=0.5,
            actions=[OpaqueFunction(function=check_ttyur)]
        ),
        trigger_external_control
    ])



  
    