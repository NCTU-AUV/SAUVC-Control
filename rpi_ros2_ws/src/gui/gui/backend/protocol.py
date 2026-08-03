"""Shared websocket protocol constants for the GUI package."""

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

CONTROLLER_GROUP_DEPTH_CONTROL = "depth_control"

CONTROLLER_ACTION_ENABLE = "enable"
CONTROLLER_ACTION_DISABLE = "disable"
CONTROLLER_ACTION_RESET = "reset"
CONTROLLER_ACTION_SET_PID_PARAMS = "set_pid_params"

SUPERVISOR_SERVICE_DEPTH_HOLD = "depth_hold"
SUPERVISOR_SERVICE_DISABLE_DEPTH_HOLD = "disable_depth_hold"
SUPERVISOR_SERVICE_AUTONOMOUS = "autonomous"
SUPERVISOR_SERVICE_DISABLE_AUTONOMOUS = "disable_autonomous"
SUPERVISOR_SERVICE_RESET_CONTROLLERS = "reset_controllers"
SUPERVISOR_SERVICE_SAFE_DISABLED = "safe_disabled"
SUPERVISOR_SERVICE_MANUAL = "manual"

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


def topic_payload(topic_name, msg):
    """Build a websocket topic payload."""
    return {
        FIELD_TYPE: TYPE_TOPIC,
        FIELD_DATA: {
            FIELD_TOPIC_NAME: topic_name,
            FIELD_MSG: msg,
        },
    }
