"""載具控制堆疊的唯一啟動入口。

    實機：ros2 launch orca_bringup bringup.launch.py
    模擬：ros2 launch orca_bringup bringup.launch.py sim:=true

刻意做成「一個 launch 檔 + 一個 sim 開關」，而不是實機／模擬各一份：
舊的 simulation_control.launch.py 是 orca_bringup.launch.py 的複製貼上分支，
兩邊已經開始各自漂移（wrench_sum 的來源清單就已經不一致）。現在兩者共用
同一份節點定義，差異只有兩處，而且都很明確：

1. 硬體專屬節點只在 sim=false 時啟動（micro_ros_agent、stm32_flasher_node、
   thruster_initialization_node、thruster_force_to_pwm_output_signal_node）。
   模擬的這段 I/O 由 SAUVC-Simulation 的 ros_gz_bridge 接手，分配層只到
   「每顆推進器的力」為止。
2. sim=true 時額外疊上 config/sim_overrides.yaml。

所有參數都來自 config/ 底下的 YAML，不寫在這個檔案裡。裝置路徑來自環境變數
（見 repo 根目錄的 .env），因為 compose 也要用同一份值去掛裝置。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    config_dir = os.path.join(get_package_share_directory('orca_bringup'), 'config')
    sim = _as_bool(LaunchConfiguration('sim').perform(context))
    namespace = LaunchConfiguration('namespace')

    # 疊加順序即優先順序，後面的覆蓋前面的。
    params = [
        os.path.join(config_dir, 'orca_params.yaml'),
        os.path.join(config_dir, 'hardware.yaml'),
    ]
    if sim:
        params.append(os.path.join(config_dir, 'sim_overrides.yaml'))

    def node(package, executable, name, **kwargs):
        return Node(
            package=package,
            executable=executable,
            namespace=namespace,
            name=name,
            parameters=params,
            output='screen',
            **kwargs,
        )

    # --- 感測邊界 ------------------------------------------------------------
    common = [
        node('sensors', 'float32_to_float64_converter_node',
             'float32_to_float64_converter_node',
             remappings=[
                 ('converters/float32_input', 'sensors/depth_m'),
                 ('converters/float64_output', 'state/depth_m'),
             ]),
        node('sensors', 'imu_to_orientation_node', 'imu_to_orientation_node'),

        # --- 控制 -------------------------------------------------------------
        # generic_pid_controller_node 靠 remap 複用成具名實例；
        # 參數 YAML 的 key 對應的是這裡的 name，不是 executable。
        node('control', 'generic_pid_controller_node', 'depth_pid_controller_node',
             remappings=[
                 ('control/pid/reference', 'control/targets/depth_m'),
                 ('control/pid/feedback', 'state/depth_m'),
                 ('control/pid/output', 'control/pid/depth/sink_force_N'),
             ]),
        node('depth_control', 'output_sink_force_to_output_wrench_node',
             'output_sink_force_to_output_wrench_node',
             remappings=[('control/wrench_command', 'control/wrench_sources/depth')]),

        # --- 致動 -------------------------------------------------------------
        node('wrench_sum', 'wrench_sum_node', 'wrench_sum_node'),
        node('thrusters', 'wrench_to_individual_thrusters_output_forces_node',
             'wrench_to_individual_thrusters_output_forces_node'),

        # --- 系統管理 ---------------------------------------------------------
        node('system_manager', 'supervisor_node', 'supervisor_node'),
        node('gui', 'gui_node', 'gui_node',
             remappings=[('control/wrench_command', 'control/wrench_sources/gui')]),
    ]

    if sim:
        return common

    # --- 以下只在實機啟動 ----------------------------------------------------
    stm32_port = LaunchConfiguration('stm32_serial_port').perform(context)
    hardware_only = [
        node('thrusters', 'thruster_initialization_node', 'thruster_initialization_node'),
        node('thrusters', 'thruster_force_to_pwm_output_signal_node',
             'thruster_force_to_pwm_output_signal_node'),
        node('stm32_manager', 'stm32_flasher_node', 'stm32_flasher_node'),
        # micro_ros_agent 不吃 ROS 參數，序列埠只能走命令列引數。
        # 預設用 /dev/serial/by-id/... 這種穩定路徑，因為搬到 Jetson 之後
        # /dev/ttyUSB* 的編號不保證與 RPi 相同。
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', stm32_port],
            output='screen',
        ),
    ]
    return common + hardware_only


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=os.environ.get('ORCA_NAMESPACE', 'orca_auv'),
            description='載具 namespace，所有 topic 的前綴',
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='true 時跳過硬體節點並疊上 sim_overrides.yaml',
        ),
        DeclareLaunchArgument(
            'stm32_serial_port',
            default_value=os.environ.get('ORCA_STM32_PORT', '/dev/ttyUSB0'),
            description='micro-ROS agent 連 STM32 的序列埠（建議用 /dev/serial/by-id/ 穩定路徑）',
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description=('是否啟動 bag_recorder 節點（提供錄製 service）。'
                         '注意這不是「開始錄」—— 錄製預設是手動的，由 GUI 或 '
                         'ros2 service call 觸發。'),
        ),
        DeclareLaunchArgument(
            'autostart_record',
            default_value='false',
            description='節點起來後立刻開始錄（比賽當天建議 true）',
        ),
        DeclareLaunchArgument(
            'record_images',
            default_value='false',
            description='autostart 時是否含影像。不影響 GUI 上每一趟的選擇。',
        ),
        DeclareLaunchArgument(
            'video_server',
            default_value='true',
            description='是否啟動 web_video_server（GUI 的影像串流來源）',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('orca_bringup'), 'launch', 'record.launch.py',
                ])
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'record': LaunchConfiguration('record'),
                'autostart': LaunchConfiguration('autostart_record'),
                'record_images': LaunchConfiguration('record_images'),
            }.items(),
        ),

        # web_video_server 是全域節點（不進 namespace），自己去 graph 上找影像。
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            condition=IfCondition(LaunchConfiguration('video_server')),
        ),

        OpaqueFunction(function=_launch_setup),
    ])
