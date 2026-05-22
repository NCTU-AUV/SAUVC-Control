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
            package='control',
            namespace=namespace,
            executable='generic_pid_controller_node',
            name='depth_pid_controller_node',
            remappings=[
                ('control/pid/reference', 'control/targets/depth_m'),
                ('control/pid/feedback', 'state/depth_m'),
                ('control/pid/output', 'control/pid/depth/sink_force_N'),
            ],
            parameters=[{
                'proportional_gain': 40.0,
                'integral_gain': 3.0,
                'derivative_gain': 12.0,
                'derivative_smoothing_factor': 0.0,
            }],
        ),
        Node(
            package='depth_control',
            namespace=namespace,
            executable='output_sink_force_to_output_wrench_node',
            parameters=[{
                'depth_force_bias_N': 10.0,
            }],
            remappings=[
                ('control/wrench_command', 'control/wrench_sources/depth'),
            ],
        ),
        Node(
            package='depth_control',
            namespace=namespace,
            executable='float32_to_float64_converter_node',
            remappings=[
                ('converters/float32_input', 'sensors/depth_m'),
                ('converters/float64_output', 'state/depth_m'),
            ],
        ),
        Node(
            package='depth_control',
            namespace=namespace,
            executable='imu_to_orientation_node',
        ),
    ])
