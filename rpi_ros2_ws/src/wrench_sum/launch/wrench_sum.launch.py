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
            description='Robot namespace',
        ),
        Node(
            package='wrench_sum',
            executable='wrench_sum_node',
            namespace=namespace,
            name='wrench_sum_node',
            output='screen',
            parameters=[{
                # 在這裡列出你所有需要整合的 topic
                'input_topics': [
                    'control/wrench_sources/gui',
                    'control/wrench_sources/depth',
                    'control/wrench_sources/decision',
                ],
                'output_topic': 'control/wrench_command',
                'publish_rate': 30.0,
                'source_timeout_s': 0.5,
            }]
        )
    ])
