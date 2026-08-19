"""Bag 錄製。

設計取捨（見 docs/REFACTOR_PLAN.md §4.3）：

* 錄製本身由 `bag_recorder` 節點以 service 控制，不再隨 bringup 自動開始。
  改動的理由是 Web GUI —— 池邊的操作者要能按下開始、跑完按停止，這兩件事
  沒辦法用 launch 參數表達。原本「隨 bringup 自動開錄」的考量（比賽時沒有
  人會記得按錄影）保留成 `autostart` 參數，預設關閉。
* topic 清單仍然來自 config/record_topics.yaml，與程式碼分離。影像改成
  **每一趟**的選擇而不是每次開機的選擇：含影像約 1.3 MB/s、不含約 0.1 MB/s，
  這個差距大到不該綁在啟動參數上。（深度改錄灰階 JPEG 之前含影像是
  38 MB/s，其中 37 MB/s 是原始深度 —— 見 record_topics.yaml。）
* 空間檢查移到「按下開始」的時候做，而不是開機時做一次。

這個檔案原本用 ExecuteProcess 包 `ros2 bag record`，並在註解裡寫著「等到
真的需要 service 控制時再升級」。這就是那次升級 —— 底下跑的仍然是同一個
`ros2 bag record`，多出來的只有生命週期管理。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('orca_bringup'), 'config', 'record_topics.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=os.environ.get('ORCA_NAMESPACE', 'orca_auv'),
            description='Robot namespace（錄製清單裡相對 topic 的前綴）',
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description=('是否啟動 bag_recorder 節點。設 false 連錄製服務都不提供 —— '
                         '這不是「不錄」的意思，不錄是預設行為。'),
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            description=('節點啟動後立刻開始錄製。預設關閉：錄製現在是操作者的動作。'
                         '比賽當天建議設 true —— 最重要的那一趟正是最可能忘記按的那趟。'),
        ),
        DeclareLaunchArgument(
            'record_images',
            default_value='false',
            description=('autostart 時是否含影像。不影響 GUI 上的每趟選擇。'
                         '含影像約 1.3 MB/s（四路壓縮，深度是灰階 JPEG 不是原始深度）。'),
        ),
        DeclareLaunchArgument(
            'bag_dir',
            default_value=os.environ.get('ORCA_BAG_ROOT', '/root/bags'),
            description='bag 輸出根目錄',
        ),
        DeclareLaunchArgument(
            'record_config',
            default_value=default_config,
            description='錄製設定 YAML 路徑',
        ),
        Node(
            package='orca_bringup',
            executable='bag_recorder_node',
            name='bag_recorder',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            condition=IfCondition(LaunchConfiguration('record')),
            parameters=[{
                'record_config': ParameterValue(
                    LaunchConfiguration('record_config'), value_type=str),
                'bag_dir': ParameterValue(
                    LaunchConfiguration('bag_dir'), value_type=str),
                'robot_namespace': ParameterValue(
                    LaunchConfiguration('namespace'), value_type=str),
                # value_type 一定要寫出來。launch 對 autostart:=false 這種
                # 字面值做型別推斷會送出字串，而節點宣告的是 bool，結果是
                # InvalidParameterTypeException 讓節點在啟動時就死掉。
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'), value_type=bool),
                'autostart_images': ParameterValue(
                    LaunchConfiguration('record_images'), value_type=bool),
            }],
        ),
    ])
