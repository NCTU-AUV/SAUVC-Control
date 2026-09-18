"""載具控制堆疊的唯一啟動入口。

    實機：ros2 launch orca_bringup bringup.launch.py
    模擬：ros2 launch orca_bringup bringup.launch.py sim:=true

刻意做成「一個 launch 檔 + 一個 sim 開關」，而不是實機／模擬各一份：
舊的 simulation_control.launch.py 是 orca_bringup.launch.py 的複製貼上分支，
兩邊已經開始各自漂移（wrench_sum 的來源清單就已經不一致）。現在兩者共用
同一份節點定義，差異只有兩處，而且都很明確：

1. 硬體專屬節點只在 sim=false 時啟動（thruster_initialization_node、
   thruster_force_to_pwm_output_signal_node，以及所選後端的 I/O 節點）。
   模擬的這段 I/O 由 SAUVC-Simulation 的 ros_gz_bridge 接手，分配層只到
   「每顆推進器的力」為止。
2. sim=true 時額外疊上 config/sim_overrides.yaml。

另有 thruster_backend 開關，決定誰把 thrusters/pwm_us 變成真的波形：

    stm32   （預設）micro_ros_agent + stm32_flasher_node，原本的 STM32 路徑
    mavlink thruster_pwm_to_mavlink_servo_output_node，送給 ArduSub 飛控

    ros2 launch orca_bringup bringup.launch.py thruster_backend:=mavlink

上游（分配層、力→PWM、ESC 初始化）兩者完全共用，所以要換回去只是改這個開關。
兩個後端都訂閱 thrusters/pwm_us，**不要同時啟動**，會有兩個東西搶著驅動推進器。

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

    def node(package, executable, name, extra_parameters=None, **kwargs):
        return Node(
            package=package,
            executable=executable,
            namespace=namespace,
            name=name,
            parameters=params + (extra_parameters or []),
            output='screen',
            **kwargs,
        )

    # 推進器輸出後端。'stm32' 是原本的 micro-ROS 路徑，'mavlink' 走新的
    # ArduSub 飛控。上游（分配層、力→PWM、ESC 初始化）兩者共用，
    # 差別只在誰訂閱 thrusters/pwm_us 去產生波形。
    thruster_backend = LaunchConfiguration('thruster_backend').perform(context)
    if thruster_backend not in ('stm32', 'mavlink'):
        raise RuntimeError(
            f"thruster_backend 只能是 'stm32' 或 'mavlink'，收到 '{thruster_backend}'"
        )

    # 走 mavlink 時 stm32_flasher_node 不會啟動，supervisor 的開機自動燒錄
    # 會卡在等一個不存在的 service 直到逾時，所以這裡直接關掉。
    supervisor_extra_parameters = (
        [] if thruster_backend == 'stm32' else [{'auto_flash_stm32_on_startup': False}]
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
        node('system_manager', 'supervisor_node', 'supervisor_node',
             extra_parameters=supervisor_extra_parameters),
        node('gui', 'gui_node', 'gui_node',
             remappings=[('control/wrench_command', 'control/wrench_sources/gui')]),
    ]

    if sim:
        return common

    # --- 以下只在實機啟動 ----------------------------------------------------
    # 力 → PWM 與 ESC 初始化兩個後端都要，它們只負責把 thrusters/pwm_us 生出來。
    hardware_only = [
        node('thrusters', 'thruster_initialization_node', 'thruster_initialization_node'),
        node('thrusters', 'thruster_force_to_pwm_output_signal_node',
             'thruster_force_to_pwm_output_signal_node'),
    ]

    if thruster_backend == 'stm32':
        stm32_port = LaunchConfiguration('stm32_serial_port').perform(context)
        hardware_only += [
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
    else:
        flight_controller_port = LaunchConfiguration('flight_controller_port').perform(context)
        hardware_only += [
            # 裝置路徑從 .env 來（compose 也要用同一份值去掛裝置），
            # 其餘參數在 hardware.yaml。
            node('thrusters', 'thruster_pwm_to_mavlink_servo_output_node',
                 'thruster_pwm_to_mavlink_servo_output_node',
                 extra_parameters=[{'mavlink_device': flight_controller_port}]),
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
            'thruster_backend',
            default_value=os.environ.get('ORCA_THRUSTER_BACKEND', 'stm32'),
            description=('推進器輸出後端：stm32（micro-ROS，原本的路徑）或 '
                         'mavlink（ArduSub 飛控）。預設仍是 stm32，'
                         '因為 mavlink 路徑還沒在實機驗證過。'),
        ),
        DeclareLaunchArgument(
            'flight_controller_port',
            default_value=os.environ.get('ORCA_FLIGHT_CONTROLLER_PORT', '/dev/ttyACM0'),
            description='ArduSub 飛控的 USB 序列埠（建議用 /dev/serial/by-id/ 穩定路徑）',
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
