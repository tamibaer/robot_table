from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

COMMON = {
    'enable_color': True,
    'enable_depth': False,
    'enable_infra1': False,
    'enable_infra2': False,
    'enable_gyro': False,
    'enable_accel': False,
    'pointcloud.enable': False,
    'align_depth.enable': False,
}

D415 = {
    **COMMON,
    'camera_name': 'D415',
    'camera_namespace': 'camera',
    'serial_no': '241222063543',
    'rgb_camera.color_profile': '640x480x30',
}

D405 = {
    **COMMON,
    'camera_name': 'D405',
    'camera_namespace': 'camera',
    'serial_no': '128422272705',
    'depth_module.color_profile': '640x480x30',
}


def generate_launch_description():
    container = ComposableNodeContainer(
        name='realsense_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='D415',
                namespace='camera',
                parameters=[D415],
            ),
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='D405',
                namespace='camera',
                parameters=[D405],
            ),
        ],
        output='screen',
    )
    return LaunchDescription([container])