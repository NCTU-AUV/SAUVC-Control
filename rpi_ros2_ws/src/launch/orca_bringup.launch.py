from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    thruster_pkg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('thrusters'),
            'launch',
            'thrusters.launch.py'
        ])),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )

    wrench_sum_node = Node(
        package='wrench_sum',
        executable='wrench_sum_node',
        namespace=namespace,
        name='wrench_sum_node',
        parameters=[{
            # wrench 匯流排的來源清單。
            # bottom_camera 已隨光流鏈移入 legacy，來源移除。
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

    gui_node = Node(
        package='gui',
        executable='gui_node',
        namespace=namespace,
        remappings=[
            ('control/wrench_command', 'control/wrench_sources/gui')
        ]
    )

    supervisor_node = Node(
        package='system_manager',
        executable='supervisor_node',
        namespace=namespace,
        name='supervisor_node',
    )

    depth_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('depth_control'),
                'launch',
                'depth_control_launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0']
    )

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
    )

    stm32_flasher_node = Node(
        package='stm32_manager',
        executable='stm32_flasher_node',
        namespace=namespace,
        name='stm32_flasher_node',
    )

    record_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('orca_bringup'),
                'launch',
                'record.launch.py',
            ])
        ),
        launch_arguments={
            'namespace': namespace,
            'record': LaunchConfiguration('record'),
            'record_images': LaunchConfiguration('record_images'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='orca_auv',
            description='Robot namespace',
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description='Whether to record a bag alongside the control stack',
        ),
        DeclareLaunchArgument(
            'record_images',
            default_value='false',
            description='Whether to also record compressed image topics',
        ),
        record_launch,
        depth_control_launch,
        thruster_pkg_launch,
        wrench_sum_node,
        supervisor_node,
        gui_node,
        stm32_flasher_node,
        micro_ros_agent,
        web_video_server,
    ])
