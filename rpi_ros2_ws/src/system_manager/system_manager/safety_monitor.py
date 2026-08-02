from rclpy.node import Node


class SafetyMonitor:
    def __init__(self, node: Node):
        self._node = node
        self.killed = False
        self.have_killed_state = False
        self.thrusters_enabled = False
        self.have_thrusters_enabled = False
        self.last_depth_stamp = None

    def update_killed(self, killed: bool):
        self.killed = killed
        self.have_killed_state = True

    def update_thrusters_enabled(self, enabled: bool):
        self.thrusters_enabled = enabled
        self.have_thrusters_enabled = True

    def update_depth(self):
        self.last_depth_stamp = self._node.get_clock().now()

    def safety_ready(self):
        if self._require_not_killed():
            if not self.have_killed_state:
                return False, "Killed state is unknown"
            if self.killed:
                return False, "Killed"

        if self._node.get_parameter("require_thrusters_enabled").value:
            if not self.have_thrusters_enabled:
                return False, "Thruster enabled state is unknown"
            if not self.thrusters_enabled:
                return False, "Thrusters are disabled"

        return True, ""

    def require_not_killed(self):
        return self._require_not_killed()

    def depth_ready(self):
        timeout_s = float(self._node.get_parameter("depth_sensor_timeout_s").value)
        if self.last_depth_stamp is None:
            return False, "Depth sensor data has not been received"
        if self._age_s(self.last_depth_stamp) > timeout_s:
            return False, "Depth sensor data is stale"
        return True, ""

    def _require_not_killed(self):
        return self._node.get_parameter("require_not_killed").value

    def _age_s(self, stamp):
        return (self._node.get_clock().now() - stamp).nanoseconds / 1e9
