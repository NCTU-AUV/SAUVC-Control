"""模擬模式的控制堆疊。

與 orca_bringup.launch.py 的差異只有兩處，其餘節點與實機共用同一份程式碼：

1. 不啟動硬體專屬節點（micro_ros_agent、stm32_flasher_node、
   thruster_force_to_pwm_output_signal_node、thruster_initialization_node）——
   這些 I/O 由 SAUVC-Simulation 的 ros_gz_bridge 對接 Gazebo。
   分配層只到 wrench_to_individual_thrusters_output_forces_node（輸出每顆推進器的力）。
2. 放寬安全門檻：模擬環境沒有實體 kill switch 與需要等待初始化的 ESC。

注意：模擬路徑因為跳過了 PWM 轉換節點，目前**完全沒有推力限幅**，
而實機的限幅寫在該節點裡。這代表模擬調出來的增益在實機會因飽和而表現不同。
限幅抽成獨立節點後這個落差就會消失（見 docs/REFACTOR_PLAN.md §7.3）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

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
        Node(
            package='wrench_sum',
            executable='wrench_sum_node',
            namespace=namespace,
            name='wrench_sum_node',
            parameters=[{
                'input_topics': [
                    'control/wrench_sources/gui',
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
