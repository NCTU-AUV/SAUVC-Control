"""Service-controlled bag recording.

Replaces the `ExecuteProcess` wrapper that used to live in record.launch.py.
That comment predicted this: recording had no project-specific logic, so a
plain process was the right call until something needed to control it. The Web
GUI is that something — an operator at the poolside wants to start a run and
stop it, and neither is expressible as a launch argument.

The process is still `ros2 bag record`. What this node adds is a lifecycle:

* start/stop over services, so the GUI is a caller rather than the owner. A
  browser reload, or a gui_node restart, does not interrupt a recording.
* the topic list resolved from record_topics.yaml at start time, with images
  as a per-run choice instead of a launch argument. Images are ~38 MB/s against
  ~0.1 MB/s without them, so "which run do I want pictures for" is a decision
  that has to be made per run, not per boot.
* SIGINT rather than SIGKILL on stop, and a wait for the process to exit, so
  metadata.yaml actually gets written. Without it every bag needs
  `ros2 bag reindex` before ros2 tooling will open it.
"""

import json
import os
import signal
import shutil
import subprocess
from datetime import datetime

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class BagRecorder(Node):

    def __init__(self):
        super().__init__('bag_recorder')

        self.declare_parameter('record_config', '')
        self.declare_parameter('bag_dir', os.environ.get('ORCA_BAG_ROOT', '/root/bags'))
        self.declare_parameter('robot_namespace',
                               os.environ.get('ORCA_NAMESPACE', 'orca_auv'))
        # Off by default: recording is now an operator action. Left as a
        # parameter because the competition case is the opposite — nobody
        # remembers to press record on the day, so the run that matters most is
        # the one most likely to go unrecorded. Set it true for a meet.
        self.declare_parameter('autostart', False)
        self.declare_parameter('autostart_images', False)

        self._proc = None
        self._bag_name = None
        self._include_images = False
        self._topic_count = 0
        self._started_at = None
        self._last_error = ''

        self._status_pub = self.create_publisher(String, '~/status', 10)
        self._start_srv = self.create_service(SetBool, '~/start', self._on_start)
        self._stop_srv = self.create_service(Trigger, '~/stop', self._on_stop)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'Bag recorder ready. bag_dir={self._param("bag_dir")} '
            f'config={self._param("record_config") or "(未設定)"}')

        if self.get_parameter('autostart').value:
            images = bool(self.get_parameter('autostart_images').value)
            ok, message = self._start(images)
            self.get_logger().info(f'autostart: {message}') if ok else \
                self.get_logger().warning(f'autostart 失敗: {message}')

    # ── helpers ──────────────────────────────────────────────────────

    def _param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _load_config(self):
        path = self._param('record_config')
        if not path or not os.path.isfile(path):
            raise FileNotFoundError(f'找不到錄製設定檔: {path or "(未設定)"}')
        with open(path, 'r', encoding='utf-8') as handle:
            return yaml.safe_load(handle) or {}

    def _resolve_topics(self, config, include_images):
        """Same rules record.launch.py used: relative names get the namespace."""
        ns = self._param('robot_namespace').strip('/')
        prefix = f'/{ns}' if ns else ''

        topics = [f'{prefix}/{t.lstrip("/")}' for t in config.get('topics') or []]
        topics += list(config.get('absolute_topics') or [])
        if include_images:
            topics += [f'{prefix}/{t.lstrip("/")}'
                       for t in config.get('image_topics') or []]
            topics += list(config.get('absolute_image_topics') or [])
        return topics

    def _missing_publishers(self, topics):
        """Topics in the list that nothing is publishing right now.

        Not fatal — `--include-unpublished-topics` records them empty — but the
        image feeds only exist when the autonomy stack was launched with
        record_images:=true, and silently recording four empty topics is
        exactly the failure this whole area already had once.
        """
        live = {name for name, _ in self.get_topic_names_and_types()}
        return [t for t in topics if t not in live]

    # ── start / stop ─────────────────────────────────────────────────

    def _start(self, include_images):
        if self._proc is not None and self._proc.poll() is None:
            return False, f'已經在錄製: {self._bag_name}'

        try:
            config = self._load_config()
        except (OSError, yaml.YAMLError) as exc:
            return False, str(exc)

        storage = config.get('storage') or {}
        storage_id = storage.get('storage_id', 'mcap')
        max_duration = int(storage.get('max_bag_duration_s', 120))
        min_free_gb = float(storage.get('min_free_space_gb', 5.0))

        bag_dir = self._param('bag_dir')
        try:
            os.makedirs(bag_dir, exist_ok=True)
            free_gb = shutil.disk_usage(bag_dir).free / (1024 ** 3)
        except OSError as exc:
            return False, f'無法存取 bag 目錄 {bag_dir}: {exc}'

        # Checked once here and never again: the operator asked for this, and
        # stopping a run underneath them is worse than a full disk they can see
        # coming in the readout.
        if free_gb < min_free_gb:
            return False, (f'剩餘空間不足: {free_gb:.1f} GB < 門檻 {min_free_gb:.1f} GB')

        topics = self._resolve_topics(config, include_images)
        if not topics:
            return False, '錄製清單是空的'

        missing = self._missing_publishers(topics)

        self._bag_name = datetime.now().strftime('orca_%Y%m%d_%H%M%S')
        output = os.path.join(bag_dir, self._bag_name)
        command = [
            'ros2', 'bag', 'record',
            '--storage', storage_id,
            '--max-bag-duration', str(max_duration),
            '--output', output,
            '--include-unpublished-topics',
        ] + topics

        try:
            # Its own session so stop() can signal the whole process group:
            # `ros2 bag record` is a wrapper, and SIGINT to the wrapper alone
            # does not always reach the recorder that owns the open file.
            self._proc = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True)
        except OSError as exc:
            self._proc = None
            return False, f'啟動 ros2 bag record 失敗: {exc}'

        self._include_images = include_images
        self._topic_count = len(topics)
        self._started_at = self.get_clock().now()
        self._last_error = ''

        message = (f'開始錄製 {self._bag_name}（{len(topics)} 條 topic'
                   f'{"，含影像" if include_images else ""}，剩餘 {free_gb:.1f} GB）')
        if missing:
            preview = ', '.join(missing[:3]) + ('…' if len(missing) > 3 else '')
            message += f'。注意：{len(missing)} 條目前沒有發布者（{preview}）'
            self.get_logger().warning(f'沒有發布者的 topic: {missing}')
        self.get_logger().info(message)
        return True, message

    def _stop(self):
        if self._proc is None or self._proc.poll() is not None:
            self._proc = None
            return False, '目前沒有在錄製'

        name = self._bag_name
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
        except OSError as exc:
            return False, f'送出 SIGINT 失敗: {exc}'

        # Waiting matters: metadata.yaml is written during the shutdown that
        # SIGINT starts. Kill before it finishes and every ros2 bag command
        # needs a reindex first.
        try:
            self._proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self.get_logger().warning('錄製器 20 秒內沒有結束，改送 SIGTERM')
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
                self._proc.wait(timeout=10)
            except (OSError, subprocess.TimeoutExpired):
                self.get_logger().error('錄製器沒有回應，bag 可能缺 metadata.yaml，'
                                        '用 ros2 bag reindex <dir> -s mcap 修復')

        self._proc = None
        bag_path = os.path.join(self._param('bag_dir'), name or '')
        has_metadata = os.path.isfile(os.path.join(bag_path, 'metadata.yaml'))
        size_mb = self._dir_size_mb(bag_path)

        message = f'已停止 {name}（{size_mb:.0f} MB）'
        if not has_metadata:
            message += '。metadata.yaml 沒寫出，需要 ros2 bag reindex'
        self.get_logger().info(message)
        return True, message

    @staticmethod
    def _dir_size_mb(path):
        try:
            return sum(os.path.getsize(os.path.join(path, f))
                       for f in os.listdir(path)
                       if os.path.isfile(os.path.join(path, f))) / (1024 ** 2)
        except OSError:
            return 0.0

    # ── service callbacks ────────────────────────────────────────────

    def _on_start(self, request, response):
        response.success, response.message = self._start(bool(request.data))
        if not response.success:
            self._last_error = response.message
        return response

    def _on_stop(self, request, response):
        response.success, response.message = self._stop()
        if not response.success:
            self._last_error = response.message
        return response

    # ── status ───────────────────────────────────────────────────────

    def _publish_status(self):
        recording = self._proc is not None and self._proc.poll() is None
        if not recording and self._proc is not None:
            # Exited on its own — a full disk is the usual reason.
            self.get_logger().warning(
                f'錄製器自行結束（exit={self._proc.returncode}）')
            self._last_error = f'錄製器自行結束，exit={self._proc.returncode}'
            self._proc = None

        bag_dir = self._param('bag_dir')
        try:
            free_gb = round(shutil.disk_usage(bag_dir).free / (1024 ** 3), 1)
        except OSError:
            free_gb = None

        elapsed = None
        if recording and self._started_at is not None:
            elapsed = round((self.get_clock().now() - self._started_at).nanoseconds / 1e9, 1)

        payload = {
            'recording': recording,
            'bag_name': self._bag_name if recording else None,
            'include_images': self._include_images if recording else False,
            'topic_count': self._topic_count if recording else 0,
            'elapsed_s': elapsed,
            'size_mb': round(self._dir_size_mb(
                os.path.join(bag_dir, self._bag_name)), 1) if recording and self._bag_name else None,
            'free_gb': free_gb,
            'error': self._last_error,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    def destroy_node(self):
        if self._proc is not None and self._proc.poll() is None:
            self.get_logger().info('節點結束，先關閉錄製器')
            self._stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
