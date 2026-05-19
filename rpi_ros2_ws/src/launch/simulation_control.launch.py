from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    publish_lk_debug_image = LaunchConfiguration('publish_lk_debug_image')

    bottom_camera_pid_fbc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xy_control'),
                'launch',
                'bottom_camera_pid_fbc_launch.py',
            ])
        ),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )

    depth_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('depth_control'),
                'launch',
                'depth_control_launch.py',
            ])
        ),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='orca_auv',
            description='Robot namespace',
        ),
        DeclareLaunchArgument(
            'publish_lk_debug_image',
            default_value='true',
            description='Whether to publish tile-line debug overlay images',
        ),
        bottom_camera_pid_fbc_launch,
        depth_control_launch,
        Node(
            package='xy_control',
            executable='lk_total_transform_node',
            namespace=namespace,
            name='lk_total_transform_node',
            parameters=[{
                'image_topic': 'camera/bottom/image_raw',
                'publish_debug_image': publish_lk_debug_image,
            }],
        ),
        Node(
            package='wrench_sum',
            executable='wrench_sum_node',
            namespace=namespace,
            name='wrench_sum_node',
            parameters=[{
                'input_topics': [
                    'control/wrench_sources/gui',
                    'control/wrench_sources/bottom_camera',
                    'control/wrench_sources/depth',
                    'control/wrench_sources/decision',
                ],
                'output_topic': 'control/wrench_command',
                'publish_rate': 30.0,
                'source_timeout_s': 0.5,
            }],
        ),
        Node(
            package='thrusters',
            executable='wrench_to_individual_thrusters_output_forces_node',
            namespace=namespace,
            name='wrench_to_individual_thrusters_output_forces_node',
        ),
        Node(
            package='system_manager',
            executable='supervisor_node',
            namespace=namespace,
            name='supervisor_node',
            parameters=[{
                'require_not_killed': False,
                'require_thrusters_enabled': False,
                'auto_flash_stm32_on_startup': False,
            }],
        ),
        Node(
            package='gui',
            executable='gui_node',
            namespace=namespace,
            name='gui_node',
            remappings=[
                ('control/wrench_command', 'control/wrench_sources/gui'),
            ],
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
        ),
    ])
