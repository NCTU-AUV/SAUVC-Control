import json
import os
import shutil

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Wrench
from std_srvs.srv import Trigger

from .backend import protocol
from .backend.aiohttp_server import AIOHTTPServer


class _AsyncParameterClient:
    """Minimal async parameter client for ROS 2 Humble."""

    def __init__(self, node: Node, node_name: str):
        self._set_client = node.create_client(SetParameters, f"{node_name}/set_parameters")
        self._get_client = node.create_client(GetParameters, f"{node_name}/get_parameters")

    def service_is_ready(self) -> bool:
        return self._set_client.service_is_ready()

    def get_service_is_ready(self) -> bool:
        return self._get_client.service_is_ready()

    def set_parameters(self, params):
        request = SetParameters.Request()
        request.parameters = [p.to_parameter_msg() for p in params]
        return self._set_client.call_async(request)

    def get_parameters(self, names):
        request = GetParameters.Request()
        request.names = list(names)
        return self._get_client.call_async(request)


class GUINode(Node):

    def __init__(self):
        super().__init__('gui_node')

        self._thruster_count = 8
        self._initial_pwm_output_signal_value_us = 1500
        self._pwm_output_signal_value_us = [
            self._initial_pwm_output_signal_value_us
            for _ in range(self._thruster_count)
        ]

        self.aiohttp_server = AIOHTTPServer(
            self._msg_callback,
            on_connect=self._publish_connect_snapshot,
        )
        self.aiohttp_server.start_threading()
        self._robot_namespace = self.get_namespace().strip('/')
        self._pid_param_names = (
            "proportional_gain",
            "integral_gain",
            "derivative_gain",
            "derivative_smoothing_factor",
        )
        self._pid_param_groups = {
            "depth": {
                "topic": protocol.TOPIC_DEPTH_PID_PARAMS,
                "nodes": {
                    "depth": "depth_pid_controller_node",
                },
            },
        }
        self._pid_param_state = {
            group_key: {}
            for group_key in self._pid_param_groups
        }
        self._pid_param_futures = {}
        self._node_to_pid_param_entry = {
            node_name: (group_key, axis_key)
            for group_key, group_spec in self._pid_param_groups.items()
            for axis_key, node_name in group_spec["nodes"].items()
        }

        self._killed_subscription = self.create_subscription(
                msg_type=Bool,
                topic=protocol.TOPIC_KILLED,
                callback=self._killed_callback,
                qos_profile=10
            )
        self._pressure_sensor_depth_subscriber = self.create_subscription(
                msg_type=Float32,
                topic=protocol.TOPIC_DEPTH_M,
                callback=self._pressure_sensor_depth_callback,
                qos_profile=10
            )
        self._target_depth_subscriber = self.create_subscription(
                msg_type=Float64,
                topic=protocol.TOPIC_TARGET_DEPTH_M,
                callback=self._target_depth_callback,
                qos_profile=10
            )
        self._stm32_log_subscriber = self.create_subscription(
                msg_type=String,
                topic=protocol.TOPIC_STM32_LOG,
                callback=self._stm32_log_callback,
                qos_profile=10
            )
        self._supervisor_mode_subscriber = self.create_subscription(
                msg_type=String,
                topic=protocol.TOPIC_SYSTEM_MANAGER_MODE,
                callback=self._supervisor_mode_callback,
                qos_profile=10
            )
        self._supervisor_status_subscriber = self.create_subscription(
                msg_type=String,
                topic=protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                callback=self._supervisor_status_callback,
                qos_profile=10
            )
        self._initialize_all_thrusters_client = self.create_client(
            Trigger,
            'thrusters/initialize_all',
        )
        self._flash_stm32_client = self.create_client(Trigger, '/flash_stm32')
        self._supervisor_clients = {
            protocol.SUPERVISOR_SERVICE_SAFE_DISABLED: self.create_client(
                Trigger,
                "system_manager/set_mode/safe_disabled",
            ),
            protocol.SUPERVISOR_SERVICE_MANUAL: self.create_client(
                Trigger,
                "system_manager/set_mode/manual",
            ),
            protocol.SUPERVISOR_SERVICE_DEPTH_HOLD: self.create_client(
                Trigger,
                "system_manager/set_mode/depth_hold",
            ),
            protocol.SUPERVISOR_SERVICE_RESET_CONTROLLERS: self.create_client(
                Trigger,
                "system_manager/reset_controllers",
            ),
            protocol.SUPERVISOR_SERVICE_DISABLE_DEPTH_HOLD: self.create_client(
                Trigger,
                "system_manager/disable/depth_hold",
            ),
            protocol.SUPERVISOR_SERVICE_AUTONOMOUS: self.create_client(
                Trigger,
                "system_manager/set_mode/autonomous",
            ),
            protocol.SUPERVISOR_SERVICE_DISABLE_AUTONOMOUS: self.create_client(
                Trigger,
                "system_manager/disable/autonomous",
            ),
        }

        self._pwm_output_signal_value_subscription = self.create_subscription(
            msg_type=Int32MultiArray,
            topic=protocol.TOPIC_THRUSTERS_PWM_US,
            callback=self._pwm_output_signal_value_subscription_callback,
            qos_profile=10
        )
        self._thrusters_enabled_subscription = self.create_subscription(
            msg_type=Bool,
            topic=protocol.TOPIC_THRUSTERS_ENABLED,
            callback=self._thrusters_enabled_callback,
            qos_profile=10
        )
        self._electromagnet_set_on_subscription = self.create_subscription(
            msg_type=Bool,
            topic=protocol.TOPIC_ELECTROMAGNET_ENABLED,
            callback=self._electromagnet_set_on_callback,
            qos_profile=10
        )

        self._set_pwm_output_signal_value_publisher = self.create_publisher(
            msg_type=Int32MultiArray,
            topic=protocol.TOPIC_THRUSTERS_PWM_US,
            qos_profile=10
        )
        self._electromagnet_set_on_publisher = self.create_publisher(
            msg_type=Bool,
            topic=protocol.TOPIC_ELECTROMAGNET_ENABLED,
            qos_profile=10
        )

        self._set_output_wrench_at_center_publisher = self.create_publisher(
            Wrench,
            protocol.TOPIC_WRENCH_COMMAND,
            10,
        )
        self._target_depth_publisher = self.create_publisher(
            Float64,
            protocol.TOPIC_TARGET_DEPTH_M,
            10,
        )
        self._controller_group_axes = {
            protocol.CONTROLLER_GROUP_DEPTH_CONTROL: dict(
                self._pid_param_groups["depth"]["nodes"]
            ),
        }
        self._controller_groups = {
            group_name: list(axis_nodes.values())
            for group_name, axis_nodes in self._controller_group_axes.items()
        }
        self._param_clients = {}
        self._controller_supervisor_actions = {
            (
                protocol.CONTROLLER_GROUP_DEPTH_CONTROL,
                protocol.CONTROLLER_ACTION_ENABLE,
            ): protocol.SUPERVISOR_SERVICE_DEPTH_HOLD,
            (
                protocol.CONTROLLER_GROUP_DEPTH_CONTROL,
                protocol.CONTROLLER_ACTION_DISABLE,
            ): protocol.SUPERVISOR_SERVICE_DISABLE_DEPTH_HOLD,
            (
                protocol.CONTROLLER_GROUP_DEPTH_CONTROL,
                protocol.CONTROLLER_ACTION_RESET,
            ): protocol.SUPERVISOR_SERVICE_RESET_CONTROLLERS,
        }
        self._pid_params_timer = self.create_timer(
            0.5,
            self._request_pid_params,
        )

        # --- camera streams -------------------------------------------------
        # The browser must not hardcode these: the real robot's RealSense sits
        # under a fixed /orca prefix while the simulator publishes camera and
        # depth under the vehicle namespace. Declared as parameters so a
        # different rig is a config change, not a frontend edit.
        ns = self._robot_namespace or "orca_auv"
        self.declare_parameter("camera_front_topic", "/orca/color/image_raw")
        self.declare_parameter("camera_detections_topic", "/yolov8_processed_image")
        self.declare_parameter("camera_bottom_topic", "/orca/usb_cam/image_raw")
        self.declare_parameter("camera_sim_front_topic", f"/{ns}/color/image_raw")
        self.declare_parameter("camera_sim_bottom_topic", f"/{ns}/camera/bottom/image_raw")
        self.declare_parameter("web_video_server_port", 8080)

        # --- bag recording --------------------------------------------------
        # record.launch.py wraps `ros2 bag record` in an ExecuteProcess, so
        # there is no status topic to subscribe to. Derive it instead: the
        # recorder's presence in the graph says whether it is running, and the
        # bag directory says how much has been written.
        self.declare_parameter(
            "bag_dir", os.environ.get("ORCA_BAG_DIR", "/root/bags"))
        self._bag_status_timer = self.create_timer(2.0, self._publish_bag_status)
        # Re-resolved periodically: the autonomy stack needs minutes to come
        # up, so a list sent once at connect would leave the detection view
        # permanently marked unavailable.
        self._camera_timer = self.create_timer(5.0, self._publish_camera_sources)

        self._start_mission_publisher = self.create_publisher(
            Bool,
            "/orca/decision/start_mission",
            10,
        )
        self._last_mode = None

    def _killed_callback(self, msg):
        self.aiohttp_server.send_topic(protocol.TOPIC_KILLED, msg.data)

    def _pressure_sensor_depth_callback(self, msg: Float32):
        self.aiohttp_server.send_topic(protocol.TOPIC_DEPTH_M, msg.data)

    def _target_depth_callback(self, msg: Float64):
        self.aiohttp_server.send_topic(protocol.TOPIC_TARGET_DEPTH_M, msg.data)

    def _stm32_log_callback(self, msg: String):
        self.aiohttp_server.send_topic(protocol.TOPIC_STM32_LOG, msg.data)

    def _publish_mission_enable(self, enabled: bool, note: str):
        """Start or stop the BehaviorTree in the autonomy stack.

        Cross-stack on purpose: the two containers share one ROS graph, and
        making the operator open a shell just to publish one Bool was the last
        step of the run that could not be done from the GUI.
        """
        msg = Bool()
        msg.data = bool(enabled)
        self._start_mission_publisher.publish(msg)
        self.get_logger().info(f"Published start_mission={enabled}")
        self._send_service_result(
            "start_mission" if enabled else "stop_mission", True, note)

    def _send_service_result(self, service_key: str, success: bool, message: str):
        self.aiohttp_server.send_topic(
            protocol.TOPIC_SERVICE_RESULT,
            {"service": service_key, "success": bool(success), "message": message},
        )

    def _supervisor_mode_callback(self, msg: String):
        previous = self._last_mode
        self._last_mode = msg.data
        self.aiohttp_server.send_topic(protocol.TOPIC_SYSTEM_MANAGER_MODE, msg.data)

        # Leaving autonomy must stop the mission. The supervisor only gates the
        # wrench bus, which lives in this stack; the BehaviorTree runs in the
        # other container and never hears about a mode change. Without this the
        # tree keeps ticking after STOP, Manual, or a FAULT the operator did not
        # trigger — burning its timeouts, overwriting the depth target through
        # SetDepth, and resuming from the middle of the run on the next arm.
        #
        # Driven off the mode topic rather than the button press so a fault the
        # operator never clicked is covered too.
        if (previous in protocol.AUTONOMOUS_MODES
                and msg.data not in protocol.AUTONOMOUS_MODES):
            self.get_logger().warning(
                f"Left autonomy ({previous} -> {msg.data}); stopping mission")
            self._publish_mission_enable(
                False, f"Mission stopped: vehicle left autonomy ({msg.data})")

    def _publish_connect_snapshot(self):
        """Re-send non-periodic state whenever a browser connects.

        The server's pending buffer only rescues the *first* client: it is
        drained on flush, so a reload or a second tab would come up with no
        camera list and a blank mode until the vehicle happened to publish.
        """
        self._publish_camera_sources()
        self._publish_bag_status()
        if self._last_mode is not None:
            self.aiohttp_server.send_topic(
                protocol.TOPIC_SYSTEM_MANAGER_MODE, self._last_mode)

    def _param_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _publish_camera_sources(self):
        """Tell the browser which streams exist and where to fetch them.

        Both the real and the simulated topic are listed for the two vehicle
        cameras. web_video_server serves whichever one actually has a
        publisher, and the page falls back to the other, so a single build
        works on the bench and in the simulator without a config switch.
        """
        port = self.get_parameter("web_video_server_port") \
            .get_parameter_value().integer_value
        candidates = [
            ("front", "Front camera",
             ["camera_front_topic", "camera_sim_front_topic"]),
            ("detections", "Detections",
             ["camera_detections_topic"]),
            ("bottom", "Bottom camera",
             ["camera_bottom_topic", "camera_sim_bottom_topic"]),
        ]

        # Resolve against the live graph here rather than letting the browser
        # probe. An <img> pointed at an MJPEG stream never fires load or error
        # reliably — the response is an endless multipart body — so a frontend
        # fallback cannot tell "no publisher" from "first frame still coming".
        # This node already knows which topics exist.
        live = {name for name, _ in self.get_topic_names_and_types()}
        sources = []
        for source_id, label, param_names in candidates:
            topics = [self._param_str(name) for name in param_names]
            resolved = next((t for t in topics if t in live), None)
            sources.append({
                "id": source_id,
                "label": label,
                "topic": resolved or topics[0],
                "available": resolved is not None,
            })

        self.aiohttp_server.send_topic(
            protocol.TOPIC_CAMERA_SOURCES,
            {"port": port, "sources": sources},
        )

    def _publish_bag_status(self):
        """Derive recording state from the graph plus the bag directory.

        There is no status topic to subscribe to — record.launch.py runs
        `ros2 bag record` as a plain process — so "is it recording" comes from
        the recorder node being present, and the size comes from the newest
        directory under bag_dir.
        """
        recording = any(
            name.startswith("rosbag2_recorder")
            for name, _ in self.get_node_names_and_namespaces()
        )
        bag_dir = self._param_str("bag_dir")
        bag_name = None
        size_mb = None
        free_gb = None
        try:
            free_gb = round(shutil.disk_usage(bag_dir).free / (1024 ** 3), 1)
            entries = [
                os.path.join(bag_dir, entry) for entry in os.listdir(bag_dir)
                if os.path.isdir(os.path.join(bag_dir, entry))
            ]
            if entries:
                newest = max(entries, key=os.path.getmtime)
                bag_name = os.path.basename(newest)
                total = sum(
                    os.path.getsize(os.path.join(newest, f))
                    for f in os.listdir(newest)
                    if os.path.isfile(os.path.join(newest, f))
                )
                size_mb = round(total / (1024 ** 2), 1)
        except OSError as exc:
            self.get_logger().debug(f"bag status unavailable: {exc}")

        self.aiohttp_server.send_topic(
            protocol.TOPIC_BAG_STATUS,
            {
                "recording": recording,
                "bag_name": bag_name,
                "size_mb": size_mb,
                "free_gb": free_gb,
            },
        )

    def _supervisor_status_callback(self, msg: String):
        self.aiohttp_server.send_topic(protocol.TOPIC_SYSTEM_MANAGER_STATUS, msg.data)

    def _request_pid_params(self):
        for group_key, group_spec in self._pid_param_groups.items():
            for axis_key, node_name in group_spec["nodes"].items():
                self._request_pid_params_for_node(group_key, axis_key, node_name)

    def _request_pid_params_for_node(self, group_key: str, axis_key: str, node_name: str):
        future = self._pid_param_futures.get(node_name)
        if future is not None and not future.done():
            return

        client = self._get_param_client(node_name)
        if not client.get_service_is_ready():
            return

        future = client.get_parameters(self._pid_param_names)
        self._pid_param_futures[node_name] = future
        future.add_done_callback(
            lambda f, g=group_key, a=axis_key, n=node_name: self._on_pid_params_result(
                g,
                a,
                n,
                f,
            )
        )

    def _on_pid_params_result(self, group_key: str, axis_key: str, node_name: str, future):
        self._pid_param_futures.pop(node_name, None)

        try:
            response = future.result()
        except Exception:
            return

        params = self._extract_pid_params(response.values)
        if params is None:
            return

        self._pid_param_state[group_key][axis_key] = params
        self._publish_pid_param_group(group_key)

    def _extract_pid_params(self, values):
        if len(values) != len(self._pid_param_names):
            return None

        params = {}
        for name, value in zip(self._pid_param_names, values):
            if value.type != ParameterType.PARAMETER_DOUBLE:
                return None
            params[name] = value.double_value
        return params

    def _publish_pid_param_group(self, group_key: str):
        group_spec = self._pid_param_groups[group_key]
        group_state = self._pid_param_state[group_key]

        if len(group_spec["nodes"]) == 1:
            axis_key = next(iter(group_spec["nodes"]))
            params = group_state.get(axis_key)
            if params is None:
                return
            payload = params
        else:
            payload = {
                axis_key: group_state[axis_key]
                for axis_key in group_spec["nodes"]
                if axis_key in group_state
            }
            if not payload:
                return

        self.aiohttp_server.send_topic(group_spec["topic"], payload)

    def _pwm_output_signal_value_subscription_callback(self, msg: Int32MultiArray):
        values = list(msg.data)
        if len(values) < self._thruster_count:
            values += [self._initial_pwm_output_signal_value_us] * (
                self._thruster_count - len(values)
            )
        self._pwm_output_signal_value_us = values[:self._thruster_count]
        self.aiohttp_server.send_topic(
            protocol.TOPIC_THRUSTERS_PWM_US,
            list(self._pwm_output_signal_value_us),
        )

    def _thrusters_enabled_callback(self, msg: Bool):
        self.aiohttp_server.send_topic(protocol.TOPIC_THRUSTERS_ENABLED, msg.data)

    def _electromagnet_set_on_callback(self, msg: Bool):
        self.aiohttp_server.send_topic(protocol.TOPIC_ELECTROMAGNET_ENABLED, msg.data)

    def _msg_callback(self, msg):
        try:
            msg_json_object = json.loads(msg)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Received non-JSON message: {msg}")
            return

        msg_type = msg_json_object.get(protocol.FIELD_TYPE)
        msg_data = msg_json_object.get(protocol.FIELD_DATA, {})

        if msg_type == protocol.TYPE_ACTION:
            action_name = msg_data.get(protocol.FIELD_ACTION_NAME)
            if action_name == protocol.ACTION_INITIALIZE_ALL_THRUSTERS:
                self._initialize_all_thrusters_client.call_async(Trigger.Request())
            elif action_name == protocol.ACTION_FLASH_STM32:
                self._flash_stm32()
            elif action_name == protocol.ACTION_SET_SUPERVISOR_SIMULATION_MODE:
                self._set_supervisor_simulation_mode(bool(msg_data.get("enabled")))
            elif action_name == protocol.ACTION_SET_SUPERVISOR_AUTONOMOUS_MODE:
                if bool(msg_data.get("enabled")):
                    self._call_supervisor(protocol.SUPERVISOR_SERVICE_AUTONOMOUS)
                else:
                    self._call_supervisor(protocol.SUPERVISOR_SERVICE_DISABLE_AUTONOMOUS)
            elif action_name == protocol.ACTION_SET_SUPERVISOR_MANUAL_MODE:
                if bool(msg_data.get("enabled")):
                    self._call_supervisor(protocol.SUPERVISOR_SERVICE_MANUAL)
                else:
                    self._call_supervisor(protocol.SUPERVISOR_SERVICE_SAFE_DISABLED)
            elif action_name == protocol.ACTION_SET_SUPERVISOR_DEPTH_HOLD:
                if bool(msg_data.get("enabled")):
                    self._call_supervisor(protocol.SUPERVISOR_SERVICE_DEPTH_HOLD)
                else:
                    self._call_supervisor(protocol.SUPERVISOR_SERVICE_DISABLE_DEPTH_HOLD)
            elif action_name == protocol.ACTION_SAFE_DISABLE:
                # Emergency stop. safe_disabled clears every controller group,
                # disables the controllers and deactivates the wrench bus, so
                # it is the one call that reliably zeroes thrust. It is also
                # how a latched FAULT is cleared, which is why it is never
                # gated behind a confirmation dialog.
                self._call_supervisor(protocol.SUPERVISOR_SERVICE_SAFE_DISABLED)
            elif action_name == protocol.ACTION_START_MISSION:
                self._publish_mission_enable(True, "Mission start published")
            elif action_name == protocol.ACTION_STOP_MISSION:
                self._publish_mission_enable(False, "Mission stop published")
            else:
                self.get_logger().warning(f"Unknown action request: {action_name}")

        if msg_type == protocol.TYPE_TOPIC:
            topic_name = msg_data.get(protocol.FIELD_TOPIC_NAME)
            if topic_name == protocol.TOPIC_THRUSTERS_PWM_US:
                try:
                    raw_values = msg_data[protocol.FIELD_MSG]["data"]
                    if not isinstance(raw_values, list):
                        raise TypeError
                    pwm_values = [int(value) for value in raw_values]
                except (KeyError, TypeError, ValueError):
                    self.get_logger().warning(f"Invalid PWM set message: {msg_json_object}")
                else:
                    if len(pwm_values) < self._thruster_count:
                        pwm_values += [self._initial_pwm_output_signal_value_us] * (
                            self._thruster_count - len(pwm_values)
                        )
                    pwm_values = pwm_values[:self._thruster_count]
                    pwm_array_msg = Int32MultiArray()
                    pwm_array_msg.data = pwm_values
                    self._set_pwm_output_signal_value_publisher.publish(pwm_array_msg)
                    self._pwm_output_signal_value_us = pwm_values

            if topic_name == protocol.TOPIC_WRENCH_COMMAND:
                try:
                    wrench_msg = msg_data[protocol.FIELD_MSG]
                    msg = Wrench()

                    msg.force.x = float(wrench_msg["force"]["x"])
                    msg.force.y = float(wrench_msg["force"]["y"])
                    msg.force.z = float(wrench_msg["force"]["z"])
                    msg.torque.x = float(wrench_msg["torque"]["x"])
                    msg.torque.y = float(wrench_msg["torque"]["y"])
                    msg.torque.z = float(wrench_msg["torque"]["z"])
                except (KeyError, TypeError, ValueError):
                    self.get_logger().warning(f"Invalid wrench message: {msg_json_object}")
                else:
                    self._set_output_wrench_at_center_publisher.publish(msg)

            if topic_name == protocol.TOPIC_TARGET_DEPTH_M:
                try:
                    target_depth = float(msg_data[protocol.FIELD_MSG]["data"])
                except (KeyError, TypeError, ValueError):
                    self.get_logger().warning(f"Invalid target depth message: {msg_json_object}")
                else:
                    msg = Float64()
                    msg.data = target_depth
                    self._target_depth_publisher.publish(msg)
                    self.aiohttp_server.send_topic(
                        protocol.TOPIC_TARGET_DEPTH_M,
                        target_depth,
                    )

            if topic_name == protocol.TOPIC_ELECTROMAGNET_ENABLED:
                try:
                    electromagnet_set_on = bool(msg_data[protocol.FIELD_MSG]["data"])
                except (KeyError, TypeError):
                    self.get_logger().warning(
                        f"Invalid electromagnet set message: {msg_json_object}"
                    )
                else:
                    msg = Bool()
                    msg.data = electromagnet_set_on
                    self._electromagnet_set_on_publisher.publish(msg)

        if msg_type == protocol.TYPE_CONTROLLER:
            group = msg_data.get(protocol.FIELD_GROUP)
            action = msg_data.get(protocol.FIELD_ACTION)
            if not group or group not in self._controller_groups:
                self.get_logger().warning(f"Unknown controller group: {msg_data}")
            elif action == protocol.CONTROLLER_ACTION_SET_PID_PARAMS:
                self._set_group_pid_params(group, msg_data.get(protocol.FIELD_PARAMS, {}))
            elif (group, action) in self._controller_supervisor_actions:
                service_key = self._controller_supervisor_actions[(group, action)]
                self._call_supervisor(service_key)
            else:
                self.get_logger().warning(f"Unknown controller action: {msg_data}")

        if msg_type not in protocol.MESSAGE_TYPES:
            self.get_logger().warning(f"Unknown message type: {msg_json_object}")

    def _flash_stm32(self):
        if not self._flash_stm32_client.service_is_ready():
            message = "STM32 flash service not ready."
            self.get_logger().warning(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_FLASH_STM32_STATUS,
                {"success": False, "message": message},
            )
            return

        future = self._flash_stm32_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_flash_stm32_result)

    def _on_flash_stm32_result(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            message = f"STM32 flash failed: {exc}"
            self.get_logger().error(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_FLASH_STM32_STATUS,
                {"success": False, "message": message},
            )
            return

        self.aiohttp_server.send_topic(
            protocol.TOPIC_FLASH_STM32_STATUS,
            {"success": response.success, "message": response.message},
        )

    def _call_supervisor(self, service_key: str):
        client = self._supervisor_clients.get(service_key)
        if client is None:
            self.get_logger().warning(f"Unknown supervisor service: {service_key}")
            return

        if not client.service_is_ready():
            message = f"Supervisor service not ready: {service_key}"
            self.get_logger().warning(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                message,
            )
            self._send_service_result(service_key, False, message)
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, key=service_key: self._log_supervisor_result(key, f)
        )

    def _log_supervisor_result(self, service_key: str, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            message = f"Supervisor call failed: {service_key}: {exc}"
            self.get_logger().warning(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                message,
            )
            self._send_service_result(service_key, False, message)
            return

        message = response.message
        if not response.success:
            self.get_logger().warning(
                f"Supervisor rejected {service_key}: {message}"
            )
        else:
            self.get_logger().info(
                f"Supervisor accepted {service_key}: {message}"
            )
        self.aiohttp_server.send_topic(protocol.TOPIC_SYSTEM_MANAGER_STATUS, message)
        # The outcome has to reach the browser, not just the log. A rejected
        # request — arming while latched in FAULT is the common one — otherwise
        # produced no visible effect whatsoever, so the operator saw a ticked
        # checkbox and a vehicle that did nothing.
        self._send_service_result(service_key, response.success, message)

    def _set_supervisor_simulation_mode(self, enabled: bool):
        client = self._get_param_client("supervisor_node")
        if not client.service_is_ready():
            message = "Supervisor parameter service not ready"
            self.get_logger().warning(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                message,
            )
            return

        require_hardware_safety = not enabled
        parameters = [
            Parameter(
                "require_not_killed",
                Parameter.Type.BOOL,
                require_hardware_safety,
            ),
            Parameter(
                "require_thrusters_enabled",
                Parameter.Type.BOOL,
                require_hardware_safety,
            ),
        ]
        future = client.set_parameters(parameters)
        future.add_done_callback(
            lambda f, mode_enabled=enabled: self._log_supervisor_config_result(
                mode_enabled,
                f,
            )
        )

    def _log_supervisor_config_result(self, simulation_mode_enabled: bool, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            message = f"Supervisor simulation mode update failed: {exc}"
            self.get_logger().warning(message)
            self.aiohttp_server.send_topic(
                protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                message,
            )
            return

        for res in response.results:
            if not res.successful:
                message = f"Supervisor simulation mode rejected: {res.reason}"
                self.get_logger().warning(message)
                self.aiohttp_server.send_topic(
                    protocol.TOPIC_SYSTEM_MANAGER_STATUS,
                    message,
                )
                return

        mode = "enabled" if simulation_mode_enabled else "disabled"
        message = f"Supervisor simulation mode {mode}"
        self.get_logger().info(message)
        self.aiohttp_server.send_topic(
            protocol.TOPIC_SYSTEM_MANAGER_STATUS,
            message,
        )

    def _get_param_client(self, node_name: str):
        client = self._param_clients.get(node_name)
        if client is None:
            client = _AsyncParameterClient(self, node_name)
            self._param_clients[node_name] = client
        return client

    def _log_param_result(self, node_name: str, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to set params on {node_name}: {exc}")
            return
        for res in response.results:
            if not res.successful:
                self.get_logger().warning(f"Param set failed on {node_name}: {res.reason}")
                return
        pid_param_entry = self._node_to_pid_param_entry.get(node_name)
        if pid_param_entry is None:
            return

        group_key, axis_key = pid_param_entry
        self._request_pid_params_for_node(group_key, axis_key, node_name)

    def _set_group_pid_params(self, group: str, params):
        nodes = self._controller_groups.get(group, [])
        if not nodes:
            self.get_logger().warning(f"No nodes configured for {group}")
            return

        axis = params.get("axis")
        if axis is not None:
            axis_node_map = self._controller_group_axes.get(group, {})
            node_name = axis_node_map.get(axis)
            if node_name is None:
                self.get_logger().warning(f"Unknown PID axis for {group}: {axis}")
                return
            nodes = [node_name]

        parsed_params = self._parse_pid_params(params)
        if parsed_params is None:
            return

        p, i, d, smoothing = parsed_params

        for node_name in nodes:
            client = self._get_param_client(node_name)
            if not client.service_is_ready():
                self.get_logger().warning(f"Parameter service not ready for {node_name}")
                continue
            parameters = [
                Parameter("proportional_gain", Parameter.Type.DOUBLE, p),
                Parameter("integral_gain", Parameter.Type.DOUBLE, i),
                Parameter("derivative_gain", Parameter.Type.DOUBLE, d),
                Parameter("derivative_smoothing_factor", Parameter.Type.DOUBLE, smoothing),
            ]
            future = client.set_parameters(parameters)
            future.add_done_callback(lambda f, n=node_name: self._log_param_result(n, f))

    def _parse_pid_params(self, params):
        try:
            p = float(params["proportional_gain"])
            i = float(params["integral_gain"])
            d = float(params["derivative_gain"])
            smoothing = float(params["derivative_smoothing_factor"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(f"Invalid PID params: {params}")
            return None

        if not (0.0 <= smoothing <= 1.0):
            self.get_logger().warning(f"derivative_smoothing_factor out of range: {smoothing}")
            return None

        return p, i, d, smoothing


def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
