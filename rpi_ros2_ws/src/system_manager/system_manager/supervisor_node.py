import rclpy
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
        self.declare_parameter("auto_flash_stm32_on_startup", True)
        self.declare_parameter("stm32_flash_service", "/flash_stm32")
        self.declare_parameter("stm32_flash_service_timeout_s", 15.0)

        self._controller_groups = {
            "depth_control": [
                "depth_pid_controller_node",
            ],
        }
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

        self._mode_publisher = self.create_publisher(String, "system_manager/mode", 10)
        self._status_publisher = self.create_publisher(String, "system_manager/status", 10)

        self.create_subscription(Bool, "sensors/killed", self._on_killed, 10)
        self.create_subscription(Bool, "thrusters/enabled", self._on_thrusters_enabled, 10)
        self.create_subscription(Float32, "sensors/depth_m", self._on_depth_float32, 10)
        self.create_subscription(Float64, "state/depth_m", self._on_depth_float64, 10)

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
        self.create_service(Trigger, "system_manager/reset_controllers", self._reset_controllers)

        self.create_timer(0.2, self._publish_state)
        self.create_timer(0.2, self._check_active_mode_safety)
        self._stm32_auto_flasher = Stm32AutoFlasher(self, self._set_status)

    def _on_killed(self, msg: Bool):
        killed = bool(msg.data)
        self._safety.update_killed(killed)
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

    def _set_safe_disabled(self, request, response):
        self._set_mode(ControlMode.SAFE_DISABLED, "Operator requested SAFE_DISABLED")
        response.success = True
        response.message = self._status
        return response

    def _set_manual(self, request, response):
        ok, reason = self._safety_ready()
        if not ok:
            self._enter_fault(reason)
            response.success = False
            response.message = reason
            return response

        self._set_mode(ControlMode.MANUAL, "Operator requested MANUAL")
        response.success = True
        response.message = self._status
        return response

    def _set_depth_hold(self, request, response):
        ok, reason = self._safety_ready()
        if ok:
            ok, reason = self._depth_ready()
        if not ok:
            self._enter_fault(reason)
            response.success = False
            response.message = reason
            return response

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

    def _reset_controllers(self, request, response):
        self._controllers.reset_all()
        response.success = True
        response.message = "Controller reset requests sent"
        return response

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
        if "depth_control" in self._active_controller_groups:
            self._mode = ControlMode.DEPTH_HOLD
            self._status = "Depth hold active"
            self._activate_wrench_sum()
        else:
            self._mode = ControlMode.SAFE_DISABLED
            self._status = "No controller groups active"
            self._deactivate_wrench_sum()

    def _safety_ready(self):
        return self._safety.safety_ready()

    def _depth_ready(self):
        return self._safety.depth_ready()

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


def main(args=None):
    rclpy.init(args=args)

    supervisor_node = SupervisorNode()
    rclpy.spin(supervisor_node)

    supervisor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
