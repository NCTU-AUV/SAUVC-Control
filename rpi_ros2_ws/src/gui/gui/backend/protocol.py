"""Shared websocket protocol constants for the GUI package.

Keep this in step with static/shared/protocol.js — the two files are the same
contract written twice, once per language.
"""

WEBSOCKET_PATH = "/websocket"
WEBSOCKET_SUBPROTOCOL = "protocolOne"

FIELD_TYPE = "type"
FIELD_DATA = "data"
FIELD_TOPIC_NAME = "topic_name"
FIELD_MSG = "msg"
FIELD_ACTION_NAME = "action_name"
FIELD_ACTION = "action"
FIELD_GROUP = "group"
FIELD_PARAMS = "params"

TYPE_ACTION = "action"
TYPE_TOPIC = "topic"
TYPE_CONTROLLER = "controller"
MESSAGE_TYPES = (TYPE_ACTION, TYPE_TOPIC, TYPE_CONTROLLER)

ACTION_INITIALIZE_ALL_THRUSTERS = "initialize_all_thrusters"
ACTION_FLASH_STM32 = "flash_stm32"
ACTION_SET_SUPERVISOR_SIMULATION_MODE = "set_supervisor_simulation_mode"
ACTION_SET_SUPERVISOR_MANUAL_MODE = "set_supervisor_manual_mode"
ACTION_SET_SUPERVISOR_AUTONOMOUS_MODE = "set_supervisor_autonomous_mode"
# depth hold used to be reachable only through the controller-group message,
# which made it the one mode the GUI could not present alongside the others.
ACTION_SET_SUPERVISOR_DEPTH_HOLD = "set_supervisor_depth_hold"
ACTION_SAFE_DISABLE = "safe_disable"
# Mission start lives in the autonomy stack. The two containers share one ROS
# graph, so publishing it from here works and saves the operator a shell.
ACTION_START_MISSION = "start_mission"
ACTION_STOP_MISSION = "stop_mission"
# Bag recording. The recorder is a node in orca_bringup with its own process
# lifecycle, so a browser reload or a gui_node restart does not interrupt a
# run in progress — this GUI is a caller, not the owner.
ACTION_START_RECORDING = "start_recording"
ACTION_STOP_RECORDING = "stop_recording"

RECORDER_SERVICE_START = "bag_recorder/start"
RECORDER_SERVICE_STOP = "bag_recorder/stop"

CONTROLLER_GROUP_DEPTH_CONTROL = "depth_control"
# Not a PID-controlled node group like depth_control above — "mission" only
# carries the one set_pool_depth action, routed straight to decision_node in
# the autonomy container rather than through _controller_groups/_get_param_client's
# per-vehicle-node fan-out.
CONTROLLER_GROUP_MISSION = "mission"

CONTROLLER_ACTION_ENABLE = "enable"
CONTROLLER_ACTION_DISABLE = "disable"
CONTROLLER_ACTION_RESET = "reset"
CONTROLLER_ACTION_SET_PID_PARAMS = "set_pid_params"
# 決賽現場池深校正。三個值（gate/drop/flare pool depth，公尺）打到 decision_node
# 的同名參數，decision_node 立刻換算並在任務進行中重發 desired_depth —— 資格
# 賽的 SetDepth 全部用字面值，不讀這幾個參數，這個動作對資格賽零影響。
CONTROLLER_ACTION_SET_POOL_DEPTH = "set_pool_depth"

SUPERVISOR_SERVICE_DEPTH_HOLD = "depth_hold"
SUPERVISOR_SERVICE_DISABLE_DEPTH_HOLD = "disable_depth_hold"
SUPERVISOR_SERVICE_AUTONOMOUS = "autonomous"
SUPERVISOR_SERVICE_DISABLE_AUTONOMOUS = "disable_autonomous"
SUPERVISOR_SERVICE_RESET_CONTROLLERS = "reset_controllers"
SUPERVISOR_SERVICE_SAFE_DISABLED = "safe_disabled"
SUPERVISOR_SERVICE_MANUAL = "manual"

# Modes in which the BehaviorTree is allowed to be flying the vehicle. Leaving
# this set has to stop the mission: the supervisor only gates the wrench bus in
# the control stack, so without an explicit stop the tree keeps ticking in the
# other container — still counting down its timeouts, still overwriting the
# depth target through SetDepth, and resuming mid-mission the moment somebody
# re-arms.
AUTONOMOUS_MODES = ("AUTONOMOUS", "AUTONOMOUS_AND_DEPTH_HOLD")

# --- vehicle topics relayed to the browser ---------------------------------
TOPIC_KILLED = "sensors/killed"
TOPIC_DEPTH_M = "sensors/depth_m"
TOPIC_STM32_LOG = "diagnostics/stm32/log"
TOPIC_SYSTEM_MANAGER_MODE = "system_manager/mode"
TOPIC_SYSTEM_MANAGER_STATUS = "system_manager/status"
TOPIC_THRUSTERS_PWM_US = "thrusters/pwm_us"
TOPIC_THRUSTERS_ENABLED = "thrusters/enabled"
TOPIC_ELECTROMAGNET_ENABLED = "actuators/electromagnet/enabled"
TOPIC_WRENCH_COMMAND = "control/wrench_command"
TOPIC_TARGET_DEPTH_M = "control/targets/depth_m"
TOPIC_DEPTH_PID_PARAMS = "control/pid/depth/gui_params"
TOPIC_FLASH_STM32_STATUS = "flash_stm32_status"

# --- GUI-only channels (never real ROS topics) -----------------------------
# Prefixed so nobody goes looking for them with `ros2 topic echo`.
#
# Service results used to go only to the node logger, so a rejected request —
# arming while latched in FAULT, for one — produced no visible effect at all:
# the checkbox stayed ticked and the operator had no way to know why nothing
# happened. Every supervisor call now reports back here.
TOPIC_SERVICE_RESULT = "gui/service_result"
# Bag recording status. Now comes straight from the bag_recorder node
# (bag_recorder/status, a JSON String) rather than being guessed from the ROS
# graph plus a directory listing — the recorder knows things the graph cannot
# show, notably whether this run includes images and why a start was refused.
TOPIC_BAG_STATUS = "gui/bag_status"
ROS_TOPIC_RECORDER_STATUS = "bag_recorder/status"
# Camera stream descriptors. The topics differ between the real robot and the
# simulator, so the browser must not hardcode them — it receives the list on
# connect and only builds web_video_server URLs from it.
TOPIC_CAMERA_SOURCES = "gui/camera_sources"
# BehaviorTree state: which node is ticking, what it is chasing, its own debug
# line. Relayed from ROS_TOPIC_MISSION_STATUS_JSON below, already decoded, so
# the browser receives an object rather than a string it has to parse again.
TOPIC_MISSION_STATUS = "decision/status"

# --- cross-stack ROS topics (absolute, outside this vehicle's namespace) -----
# The decision node runs in the autonomy container under a fixed /orca prefix.
# Both stacks share one ROS graph, so subscribing across works; what does not
# work is the message type. /orca/decision/status carries
# orca_interface/msg/DecisionStatus and orca_interface is a SAUVC-JETSON package
# that is not built into the control container, so this node cannot deserialise
# it — which is why mission state used to be missing from the GUI entirely.
# decision_node therefore mirrors the same fields as JSON in a std_msgs/String,
# a type every container already has. Starting a mission has always worked for
# the same reason: std_msgs/Bool.
ROS_TOPIC_MISSION_STATUS_JSON = "/orca/decision/status_json"
ROS_TOPIC_START_MISSION = "/orca/decision/start_mission"
# decision_node's parameter services, for set_pool_depth. Absolute and outside
# this vehicle's namespace for the same reason as the two topics above — the
# node lives in the autonomy container's fixed /orca prefix, not under
# self.get_namespace(). _get_param_client builds "<name>/set_parameters" from
# this, so the leading slash is what keeps it from resolving into our own
# namespace.
ROS_NODE_DECISION = "/decision_node"


def topic_payload(topic_name, msg):
    """Build a websocket topic payload."""
    return {
        FIELD_TYPE: TYPE_TOPIC,
        FIELD_DATA: {
            FIELD_TOPIC_NAME: topic_name,
            FIELD_MSG: msg,
        },
    }
