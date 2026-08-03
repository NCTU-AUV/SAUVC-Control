from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='orca_auv',
            description='Robot namespace'
        ),
        Node(
            package='bottom_camera',
            executable='bottom_camera_node',
            namespace=namespace,
            name='bottom_camera_node',
        ),
    ])
