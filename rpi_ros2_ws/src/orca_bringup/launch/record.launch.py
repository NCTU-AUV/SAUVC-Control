"""Bag 錄製。

設計取捨（見 docs/REFACTOR_PLAN.md §4.3）：

* 隨 bringup 自動開始錄，用 launch 參數關掉 —— 比賽時沒有人會記得按錄影。
* 啟動前檢查剩餘空間；不足就不錄並印警告，而不是把系統碟寫滿導致其他寫入失敗。
* topic 清單來自 config/record_topics.yaml，與程式碼分離。

刻意用 ExecuteProcess 包 `ros2 bag record`，而不是自己寫 recorder node：
錄製本身沒有專案特有邏輯，多一個節點就多一份要維護的東西。等到真的需要
「標記某一趟」之類的 service 控制時再升級。
"""

import os
import shutil
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _as_bool(value) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _load_config(config_path: str) -> dict:
    with open(config_path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def _resolve_topics(config: dict, namespace: str, record_images: bool) -> list:
    """把 config 裡的相對 topic 加上 namespace，絕對 topic 原樣保留。"""
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''

    topics = [f'{prefix}/{t.lstrip("/")}' for t in config.get('topics') or []]
    topics += list(config.get('absolute_topics') or [])
    if record_images:
        topics += [f'{prefix}/{t.lstrip("/")}' for t in config.get('image_topics') or []]
    return topics


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('record_config').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    output_root = LaunchConfiguration('bag_dir').perform(context)
    record_images = _as_bool(LaunchConfiguration('record_images').perform(context))

    if not os.path.isfile(config_file):
        return [LogInfo(msg=f'[record] 找不到錄製設定檔，不啟動錄製：{config_file}')]

    config = _load_config(config_file)
    storage = config.get('storage') or {}
    storage_id = storage.get('storage_id', 'mcap')
    max_duration_s = int(storage.get('max_bag_duration_s', 120))
    min_free_gb = float(storage.get('min_free_space_gb', 5.0))

    # 空間檢查要對「實際會寫入的那顆磁碟」做，所以先確保目錄存在再量。
    try:
        os.makedirs(output_root, exist_ok=True)
        free_gb = shutil.disk_usage(output_root).free / (1024 ** 3)
    except OSError as exc:
        return [LogInfo(msg=f'[record] 無法存取 bag 目錄，不啟動錄製：{output_root} ({exc})')]

    if free_gb < min_free_gb:
        return [LogInfo(
            msg=f'[record] 剩餘空間不足，不啟動錄製：{output_root} '
                f'剩 {free_gb:.1f} GB < 門檻 {min_free_gb:.1f} GB'
        )]

    topics = _resolve_topics(config, namespace, record_images)
    if not topics:
        return [LogInfo(msg='[record] 錄製清單是空的，不啟動錄製')]

    bag_name = datetime.now().strftime('orca_%Y%m%d_%H%M%S')
    output_path = os.path.join(output_root, bag_name)

    command = [
        'ros2', 'bag', 'record',
        '--storage', storage_id,
        '--max-bag-duration', str(max_duration_s),
        '--output', output_path,
        # 錄製清單裡的 topic 在啟動當下可能還沒出現（節點還在起、JETSON 還沒開），
        # 沒有這個旗標的話 rosbag2 會直接放棄那些 topic 而不是等它們出現。
        '--include-unpublished-topics',
    ] + topics

    return [
        LogInfo(msg=f'[record] 錄製到 {output_path}（{storage_id}，'
                    f'每 {max_duration_s}s 分段，{len(topics)} 個 topic，'
                    f'剩餘空間 {free_gb:.1f} GB）'),
        ExecuteProcess(cmd=command, output='screen'),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('orca_bringup'), 'config', 'record_topics.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=os.environ.get('ORCA_NAMESPACE', 'orca_auv'),
            description='Robot namespace（錄製清單的 topic 前綴）',
        ),
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description='是否錄 bag。設 false 可完全關閉。',
        ),
        DeclareLaunchArgument(
            'record_images',
            default_value='false',
            description='是否連影像一起錄（只錄 compressed）。會大幅增加 I/O 與儲存用量。',
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
        OpaqueFunction(
            function=_launch_setup,
            condition=IfCondition(LaunchConfiguration('record')),
        ),
    ])
