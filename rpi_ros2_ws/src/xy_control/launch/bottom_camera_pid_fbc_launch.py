from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    x_coordinate_pid_controller_node = Node(
        package='control',
        namespace=namespace,
        executable='generic_pid_controller_node',
        name='x_coordinate_pid_controller_node',
        remappings=[
            ('control/pid/reference', 'control/pid/bottom_camera/x/reference_px'),
            ('control/pid/feedback', 'control/pid/bottom_camera/x/feedback_px'),
            ('control/pid/output', 'control/pid/bottom_camera/x/force_world_N'),
        ],
        parameters=[{
            'proportional_gain': 0.03,
            'integral_gain': 0.00001,
            'derivative_gain': 0.001,
            'derivative_smoothing_factor': 0.2,
        }],
    )

    y_coordinate_pid_controller_node = Node(
        package='control',
        namespace=namespace,
        executable='generic_pid_controller_node',
        name='y_coordinate_pid_controller_node',
        remappings=[
            ('control/pid/reference', 'control/pid/bottom_camera/y/reference_px'),
            ('control/pid/feedback', 'control/pid/bottom_camera/y/feedback_px'),
            ('control/pid/output', 'control/pid/bottom_camera/y/force_world_N'),
        ],
        parameters=[{
            'proportional_gain': 0.03,
            'integral_gain': 0.00001,
            'derivative_gain': 0.001,
            'derivative_smoothing_factor': 0.2,
        }],
    )

    yaw_angle_pid_controller_node = Node(
        package='control',
        namespace=namespace,
        executable='generic_pid_controller_node',
        name='yaw_angle_pid_controller_node',
        remappings=[
            ('control/pid/reference', 'control/pid/bottom_camera/yaw/reference_rad'),
            ('control/pid/feedback', 'state/bottom_camera/yaw_rad'),
            ('control/pid/output', 'control/pid/bottom_camera/yaw/torque_Nm'),
        ],
        parameters=[{
            'proportional_gain': 0.5,
            'integral_gain': 0.0,
            'derivative_gain': 0.02,
            'derivative_smoothing_factor': 0.2,
        }],
    )

    yaw_reference_unwrapper_node = Node(
        package='xy_control',
        namespace=namespace,
        executable='yaw_reference_unwrapper_node',
        name='yaw_reference_unwrapper_node',
        parameters=[{
            'target_topic': 'control/targets/bottom_camera/yaw_rad',
            'feedback_topic': 'state/bottom_camera/yaw_rad',
            'output_topic': 'control/pid/bottom_camera/yaw/reference_rad',
        }],
    )

    bridge_node = Node(
        package='xy_control',
        executable='bottom_camera_pid_bridge_node',
        namespace=namespace,
        parameters=[{
            'yaw_topic': 'state/bottom_camera/yaw_rad',
            'output_topic': 'control/wrench_sources/bottom_camera',
            'x_manipulated_topic': 'control/pid/bottom_camera/x/force_world_N',
            'y_manipulated_topic': 'control/pid/bottom_camera/y/force_world_N',
            'yaw_manipulated_topic': 'control/pid/bottom_camera/yaw/torque_Nm',
            'max_force_x_N': 10.0,
            'max_force_y_N': 10.0,
            'max_yaw_torque_Nm': 5.0,
        }],
    )

    waypoint_target_publisher = Node(
        package='xy_control',
        executable='waypoint_target_publisher',
        namespace=namespace,
        name='waypoint_target_publisher',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='orca_auv',
                description='Robot namespace',
            ),
            x_coordinate_pid_controller_node,
            y_coordinate_pid_controller_node,
            yaw_reference_unwrapper_node,
            yaw_angle_pid_controller_node,
            bridge_node,
            waypoint_target_publisher,
        ]
    )
