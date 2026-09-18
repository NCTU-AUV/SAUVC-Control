import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_srvs.srv import Trigger

from system_manager.control_mode import ControlMode
from system_manager.controller_groups import ControllerGroupManager
from system_manager.safety_monitor import SafetyMonitor
from system_manager.stm32_auto_flasher import Stm32AutoFlasher


class SupervisorNode(Node):
    """Owns high-level control mode and activates controller groups."""

    def __init__(self):
        super().__init__("supervisor_node")

        self.declare_parameter("require_not_killed", True)
        self.declare_parameter("require_thrusters_enabled", True)
        self.declare_parameter("depth_sensor_timeout_s", 1.0)
        self.declare_parameter("decision_timeout_s", 1.0)
        self.declare_parameter("auto_flash_stm32_on_startup", True)
        self.declare_parameter("stm32_flash_service", "/flash_stm32")
        self.declare_parameter("stm32_flash_service_timeout_s", 15.0)
        self.declare_parameter("auto_initialize_thrusters_on_killed_recovery", True)
        self.declare_parameter("thrusters_initialize_service", "thrusters/initialize_all")
        self.declare_parameter("thrusters_initialize_service_timeout_s", 15.0)

        # 有 lifecycle 節點要啟停的群組。
        self._controller_groups = {
            "depth_control": [
                "depth_pid_controller_node",
            ],
        }
        # autonomous 沒有自己的 lifecycle 節點 —— 它放行的是 Autonomy 堆疊直接
        # 發布到 wrench 匯流排的 control/wrench_sources/decision。因此它只需要
        # wrench_sum 是 active，外加一個「決策來源還活著」的安全前提。
        self._autonomous_group = "autonomous"
        self._wrench_sum_group = "wrench_sum"

        self._mode = ControlMode.SAFE_DISABLED
        self._status = "Initialized in SAFE_DISABLED"
        self._active_controller_groups = set()
        self._safety = SafetyMonitor(self)
        self._controllers = ControllerGroupManager(
            self,
            self._controller_groups,
        )
        self._wrench_sum = ControllerGroupManager(
            self,
            {
                self._wrench_sum_group: [
                    "wrench_sum_node",
                ],
            },
        )
        self._wrench_sum_active = False
        self._last_killed_state = None
        self._have_killed_state = False
        self._auto_initialize_thrusters_in_progress = False
        self._auto_initialize_thrusters_request_pending = False
        self._auto_initialize_thrusters_start_stamp = None
        self._auto_initialize_thrusters_timer = None

        self._mode_publisher = self.create_publisher(String, "system_manager/mode", 10)
        self._status_publisher = self.create_publisher(String, "system_manager/status", 10)

        self._initialize_all_thrusters_client = self.create_client(
            Trigger,
            self.get_parameter("thrusters_initialize_service").value,
        )

        self.create_subscription(Bool, "sensors/killed", self._on_killed, 10)
        self.create_subscription(Bool, "thrusters/enabled", self._on_thrusters_enabled, 10)
        self.create_subscription(Float32, "sensors/depth_m", self._on_depth_float32, 10)
        self.create_subscription(Float64, "state/depth_m", self._on_depth_float64, 10)
        self.create_subscription(
            Wrench,
            "control/wrench_sources/decision",
            self._on_decision_wrench,
            10,
        )

        self.create_service(
            Trigger,
            "system_manager/set_mode/safe_disabled",
            self._set_safe_disabled,
        )
        self.create_service(Trigger, "system_manager/set_mode/manual", self._set_manual)
        self.create_service(Trigger, "system_manager/set_mode/depth_hold", self._set_depth_hold)
        self.create_service(
            Trigger,
            "system_manager/disable/depth_hold",
            self._disable_depth_hold,
        )
        self.create_service(
            Trigger,
            "system_manager/set_mode/autonomous",
            self._set_autonomous,
        )
        self.create_service(
            Trigger,
            "system_manager/disable/autonomous",
            self._disable_autonomous,
        )
        self.create_service(Trigger, "system_manager/reset_controllers", self._reset_controllers)

        self.create_timer(0.2, self._publish_state)
        self.create_timer(0.2, self._check_active_mode_safety)
        self._auto_initialize_thrusters_timer = self.create_timer(
            0.5,
            self._maybe_initialize_all_thrusters,
        )
        self._stm32_auto_flasher = Stm32AutoFlasher(self, self._set_status)

    def _on_killed(self, msg: Bool):
        killed = bool(msg.data)
        should_initialize_thrusters = (
            self._have_killed_state
            and self._last_killed_state is True
            and killed is False
        )
        self._last_killed_state = killed
        self._have_killed_state = True
        self._safety.update_killed(killed)
        if should_initialize_thrusters:
            self._start_auto_initialize_all_thrusters()
        if self._safety.require_not_killed() and killed:
            self._enter_fault("Killed")

    def _on_thrusters_enabled(self, msg: Bool):
        thrusters_enabled = bool(msg.data)
        self._safety.update_thrusters_enabled(thrusters_enabled)
        if not thrusters_enabled and self._active_controller_groups:
            self._enter_fault("Thrusters are disabled")

    def _on_depth_float32(self, msg: Float32):
        self._safety.update_depth()

    def _on_depth_float64(self, msg: Float64):
        self._safety.update_depth()

    def _on_decision_wrench(self, msg: Wrench):
        del msg
        self._safety.update_decision()

    def _reject(self, response, reason: str):
        """拒絕一次模式請求，不改動載具狀態。

        「我想啟用的來源還沒就緒」是請求層級的前提不滿足，不是載具故障。
        原本這裡呼叫 _enter_fault，於是在水中穩定保持深度時、只要在 Jetson
        發出第一筆 wrench 之前勾選自主，就會清掉所有控制群組、停用深度 PID
        並輸出零力 —— 載具因為一次「應該被回絕的請求」而失去深度保持。

        已啟用的模式若之後才失去前提，由 _check_active_mode_safety 週期性
        偵測並進入 FAULT；kill 開關與推進器停用也各自有獨立的 callback。
        這裡不需要、也不應該重複那件事。
        """
        self.get_logger().warning(f"Mode request rejected: {reason}")
        response.success = False
        response.message = reason
        return response

    def _fault_latched(self):
        """FAULT 中的「啟用」請求要擋的理由，不在 FAULT 則回 None。

        必須在服務處理器的**最前面**呼叫。_refresh_mode_from_active_groups
        尾端也有一道 FAULT 鎖存，但那一道是給 disable 路徑用的：它要到
        _set_depth_hold／_set_autonomous 已經 _enable_group、已經把群組加進
        _active_controller_groups 之後才生效，於是深度 PID 被拉回 ACTIVE
        （正是 _enter_fault 的 disable_all 要防止的事）、_activate_wrench_sum
        卻永遠不會執行，推力全程為零 —— 而服務仍回報 success=True。

        擋不住的原因是 safety_monitor 的三個 ready 檢查（safety_ready／
        depth_ready／decision_ready）完全不看 _mode，kill 開關放開之後它們
        就全數通過。_check_active_mode_safety 是以群組集合為判準、不是 _mode，
        所以也不會修正這個矛盾。

        FAULT 只能由操作員明確地經 SAFE_DISABLED 或 MANUAL 清除 —— 那兩條
        路徑直接呼叫 _set_mode，因此不受這道閘門限制。
        """
        if self._mode != ControlMode.FAULT:
            return None
        return (f"Vehicle is in FAULT ({self._status}); "
                "clear it with SAFE_DISABLED or MANUAL first")

    def _set_safe_disabled(self, request, response):
        self._set_mode(ControlMode.SAFE_DISABLED, "Operator requested SAFE_DISABLED")
        response.success = True
        response.message = self._status
        return response

    def _set_manual(self, request, response):
        ok, reason = self._safety_ready()
        if not ok:
            return self._reject(response, reason)

        self._set_mode(ControlMode.MANUAL, "Operator requested MANUAL")
        response.success = True
        response.message = self._status
        return response

    def _set_depth_hold(self, request, response):
        fault_reason = self._fault_latched()
        if fault_reason:
            return self._reject(response, fault_reason)

        ok, reason = self._safety_ready()
        if ok:
            ok, reason = self._depth_ready()
        if not ok:
            return self._reject(response, reason)

        if "depth_control" not in self._active_controller_groups:
            self._reset_group("depth_control")
            self._enable_group("depth_control")
            self._active_controller_groups.add("depth_control")
        self._refresh_mode_from_active_groups()
        response.success = True
        response.message = self._status
        return response

    def _disable_depth_hold(self, request, response):
        self._disable_group("depth_control")
        self._active_controller_groups.discard("depth_control")
        self._refresh_mode_from_active_groups()
        response.success = True
        response.message = self._status
        return response

    def _set_autonomous(self, request, response):
        fault_reason = self._fault_latched()
        if fault_reason:
            return self._reject(response, fault_reason)

        ok, reason = self._safety_ready()
        if ok:
            ok, reason = self._decision_ready()
        if not ok:
            return self._reject(response, reason)

        self._active_controller_groups.add(self._autonomous_group)
        self._refresh_mode_from_active_groups()
        response.success = True
        response.message = self._status
        return response

    def _disable_autonomous(self, request, response):
        self._active_controller_groups.discard(self._autonomous_group)
        self._refresh_mode_from_active_groups()
        response.success = True
        response.message = self._status
        return response

    def _reset_controllers(self, request, response):
        self._controllers.reset_all()
        response.success = True
        response.message = "Controller reset requests sent"
        return response

    def _start_auto_initialize_all_thrusters(self):
        if (
            self._auto_initialize_thrusters_in_progress
            or self._auto_initialize_thrusters_request_pending
        ):
            return

        if not self.get_parameter("auto_initialize_thrusters_on_killed_recovery").value:
            self.get_logger().info("Thruster auto-initialization disabled")
            return

        self._auto_initialize_thrusters_in_progress = True
        self._auto_initialize_thrusters_start_stamp = self.get_clock().now()
        self._set_status("Thruster auto-initialization started")
        self.get_logger().info("Thruster auto-initialization started")

    def _maybe_initialize_all_thrusters(self):
        if not self._auto_initialize_thrusters_in_progress:
            return

        if self._initialize_all_thrusters_client.service_is_ready():
            self._auto_initialize_thrusters_in_progress = False
            self._auto_initialize_thrusters_request_pending = True
            self.get_logger().info("Thruster auto-initialization request sent")
            future = self._initialize_all_thrusters_client.call_async(Trigger.Request())
            future.add_done_callback(self._on_initialize_all_thrusters_result)
            return

        timeout_s = float(self.get_parameter("thrusters_initialize_service_timeout_s").value)
        if self._age_s(self._auto_initialize_thrusters_start_stamp) > timeout_s:
            self._auto_initialize_thrusters_in_progress = False
            service_name = self.get_parameter("thrusters_initialize_service").value
            status = f"Thruster auto-initialization skipped: {service_name} service not ready"
            self._set_status(status)
            self.get_logger().warning(status)

    def _on_initialize_all_thrusters_result(self, future):
        self._auto_initialize_thrusters_request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            status = f"Thruster auto-initialization failed: {exc}"
            self._set_status(status)
            self.get_logger().warning(status)
            return

        if response.success:
            status = "Thruster auto-initialization succeeded"
            if response.message:
                status = f"{status}: {response.message}"
            self._set_status(status)
            self.get_logger().info(status)
            return

        status = "Thruster auto-initialization failed"
        if response.message:
            status = f"{status}: {response.message}"
        self._set_status(status)
        self.get_logger().warning(status)

    def _set_status(self, status: str):
        self._status = status

    def _set_mode(self, mode: ControlMode, status: str):
        self._mode = mode
        self._status = status
        if mode in (ControlMode.SAFE_DISABLED, ControlMode.MANUAL, ControlMode.FAULT):
            self._active_controller_groups.clear()
            self._controllers.disable_all()
        if mode == ControlMode.MANUAL:
            self._activate_wrench_sum()
        elif mode in (ControlMode.SAFE_DISABLED, ControlMode.FAULT):
            self._deactivate_wrench_sum()

    def _enter_fault(self, reason: str):
        if self._mode == ControlMode.FAULT and self._status == reason:
            return
        self._mode = ControlMode.FAULT
        self._status = reason
        self._active_controller_groups.clear()
        self.get_logger().warning(f"Entering FAULT: {reason}")
        self._controllers.disable_all()
        self._deactivate_wrench_sum()

    def _refresh_mode_from_active_groups(self):
        # FAULT 是鎖存狀態，只能由操作員明確地經 SAFE_DISABLED 或 MANUAL 清除
        # （那兩條路徑會直接呼叫 _set_mode）。原本這個函式的 else 分支會無條件
        # 把模式覆寫成 SAFE_DISABLED、狀態覆寫成「No controller groups active」，
        # 所以操作員在 FAULT 下按任一個 disable 鍵，就會在沒有任何安全複檢的
        # 情況下解除 FAULT，並且銷毀故障原因 —— 事後的 bag 與儀表板都失去
        # 載具為何停機的唯一記錄。
        #
        # 這一道只負責 disable 路徑（_disable_depth_hold／_disable_autonomous）。
        # 「啟用」路徑必須更早擋 —— 走到這裡時它們已經改過群組並轉換過生命週期，
        # 光是不覆寫模式救不了，見 _fault_latched。
        if self._mode == ControlMode.FAULT:
            self.get_logger().warning(
                "Controller group changed while in FAULT; keeping FAULT "
                f"(reason: {self._status})")
            return

        depth_active = "depth_control" in self._active_controller_groups
        autonomous_active = self._autonomous_group in self._active_controller_groups

        if autonomous_active and depth_active:
            self._mode = ControlMode.AUTONOMOUS_AND_DEPTH_HOLD
            self._status = "Autonomy and depth hold active"
        elif autonomous_active:
            self._mode = ControlMode.AUTONOMOUS
            self._status = "Autonomy active"
        elif depth_active:
            self._mode = ControlMode.DEPTH_HOLD
            self._status = "Depth hold active"
        else:
            self._mode = ControlMode.SAFE_DISABLED
            self._status = "No controller groups active"
            self._deactivate_wrench_sum()
            return

        self._activate_wrench_sum()

    def _safety_ready(self):
        return self._safety.safety_ready()

    def _depth_ready(self):
        return self._safety.depth_ready()

    def _decision_ready(self):
        return self._safety.decision_ready()

    def _check_active_mode_safety(self):
        if not self._active_controller_groups:
            return

        ok, reason = self._safety_ready()
        if not ok:
            self._enter_fault(reason)
            return

        if "depth_control" in self._active_controller_groups:
            ok, reason = self._depth_ready()
            if not ok:
                self._enter_fault(reason)
                return

        if self._autonomous_group in self._active_controller_groups:
            ok, reason = self._decision_ready()
            if not ok:
                self._enter_fault(reason)

    def _enable_group(self, group: str):
        self._controllers.enable_group(group)

    def _disable_group(self, group: str):
        self._controllers.disable_group(group)

    def _reset_group(self, group: str):
        self._controllers.reset_group(group)

    def _activate_wrench_sum(self):
        if self._wrench_sum_active:
            return
        self._wrench_sum.enable_group(self._wrench_sum_group)
        self._wrench_sum_active = True

    def _deactivate_wrench_sum(self):
        if not self._wrench_sum_active:
            return
        self._wrench_sum.disable_group(self._wrench_sum_group)
        self._wrench_sum_active = False

    def _publish_state(self):
        mode_msg = String()
        mode_msg.data = self._mode.value
        self._mode_publisher.publish(mode_msg)

        status_msg = String()
        status_msg.data = self._status
        self._status_publisher.publish(status_msg)

    def _age_s(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)

    supervisor_node = SupervisorNode()
    rclpy.spin(supervisor_node)

    supervisor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
