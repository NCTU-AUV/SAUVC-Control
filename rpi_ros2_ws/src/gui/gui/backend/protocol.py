"""Shared websocket protocol constants for the GUI package."""

WEBSOCKET_PATH = "/websocket"
WEBSOCKET_SUBPROTOCOL = "protocolOne"

FIELD_TYPE = "type"
FIELD_DATA = "data"
FIELD_TOPIC_NAME = "topic_name"
FIELD_MSG = "msg"
FIELD_ACTION_NAME = "action_name"
FIELD_TARGET = "target"
FIELD_ACTION = "action"
FIELD_GROUP = "group"
FIELD_PARAMS = "params"

TYPE_ACTION = "action"
TYPE_TOPIC = "topic"
TYPE_PROCESS = "process"
TYPE_CONTROLLER = "controller"
MESSAGE_TYPES = (TYPE_ACTION, TYPE_TOPIC, TYPE_PROCESS, TYPE_CONTROLLER)

ACTION_INITIALIZE_ALL_THRUSTERS = "initialize_all_thrusters"
ACTION_FLASH_STM32 = "flash_stm32"
ACTION_SET_SUPERVISOR_SIMULATION_MODE = "set_supervisor_simulation_mode"
ACTION_SET_SUPERVISOR_MANUAL_MODE = "set_supervisor_manual_mode"
ACTION_MOVE_TO_POINT = "move_to_point"
ACTION_CANCEL_MOVE_TO_POINT = "cancel_move_to_point"
ACTION_RESET_BOTTOM_CAMERA_POSE = "reset_bottom_camera_pose"

PROCESS_BOTTOM_CAMERA_PID_FBC_LAUNCH = "bottom_camera_pid_fbc_launch"
PROCESS_DEPTH_CONTROL_LAUNCH = "depth_control_launch"

CONTROLLER_GROUP_BOTTOM_CAMERA_PID_FBC = "bottom_camera_pid_fbc"
CONTROLLER_GROUP_DEPTH_CONTROL = "depth_control"

CONTROLLER_ACTION_ENABLE = "enable"
CONTROLLER_ACTION_DISABLE = "disable"
CONTROLLER_ACTION_RESET = "reset"
CONTROLLER_ACTION_SET_PID_PARAMS = "set_pid_params"

PROCESS_ACTION_START = "start"
PROCESS_ACTION_STOP = "stop"
PROCESS_ACTION_KILL = "kill"

SUPERVISOR_SERVICE_BOTTOM_CAMERA_HOLD = "bottom_camera_hold"
SUPERVISOR_SERVICE_DEPTH_HOLD = "depth_hold"
SUPERVISOR_SERVICE_DISABLE_BOTTOM_CAMERA_HOLD = "disable_bottom_camera_hold"
SUPERVISOR_SERVICE_DISABLE_DEPTH_HOLD = "disable_depth_hold"
SUPERVISOR_SERVICE_RESET_CONTROLLERS = "reset_controllers"
SUPERVISOR_SERVICE_SAFE_DISABLED = "safe_disabled"
SUPERVISOR_SERVICE_MANUAL = "manual"

TOPIC_KILLED = "sensors/killed"
TOPIC_DEPTH_M = "sensors/depth_m"
TOPIC_STM32_LOG = "diagnostics/stm32/log"
TOPIC_SYSTEM_MANAGER_MODE = "system_manager/mode"
TOPIC_SYSTEM_MANAGER_STATUS = "system_manager/status"
TOPIC_BOTTOM_CAMERA_POSE_PX = "camera/bottom/pose_px"
TOPIC_BOTTOM_CAMERA_IMAGE_RAW = "camera/bottom/image_raw"
TOPIC_BOTTOM_CAMERA_PID_X_REFERENCE_PX = "control/pid/bottom_camera/x/reference_px"
TOPIC_BOTTOM_CAMERA_PID_Y_REFERENCE_PX = "control/pid/bottom_camera/y/reference_px"
TOPIC_BOTTOM_CAMERA_YAW_TARGET_RAD = "control/targets/bottom_camera/yaw_rad"
TOPIC_BOTTOM_CAMERA_PID_YAW_REFERENCE_RAD = "control/pid/bottom_camera/yaw/reference_rad"
TOPIC_BOTTOM_CAMERA_PID_X_FEEDBACK_PX = "control/pid/bottom_camera/x/feedback_px"
TOPIC_BOTTOM_CAMERA_PID_Y_FEEDBACK_PX = "control/pid/bottom_camera/y/feedback_px"
TOPIC_BOTTOM_CAMERA_PID_YAW_FEEDBACK_RAD = "state/bottom_camera/yaw_rad"
TOPIC_BOTTOM_CAMERA_PID_PARAMS = "control/pid/bottom_camera/gui_params"
TOPIC_BOTTOM_CAMERA_TOPIC_STATS = "camera/bottom/gui/topic_stats"
TOPIC_THRUSTERS_PWM_US = "thrusters/pwm_us"
TOPIC_THRUSTERS_ENABLED = "thrusters/enabled"
TOPIC_ELECTROMAGNET_ENABLED = "actuators/electromagnet/enabled"
TOPIC_WRENCH_COMMAND = "control/wrench_command"
TOPIC_TARGET_DEPTH_M = "control/targets/depth_m"
TOPIC_DEPTH_PID_PARAMS = "control/pid/depth/gui_params"
TOPIC_FLASH_STM32_STATUS = "flash_stm32_status"
TOPIC_MOVE_TO_POINT_STATUS = "control/targets/move_to_point/gui_status"
TOPIC_BOTTOM_CAMERA_POSE_RESET_STATUS = "camera/bottom/reset_pose/gui_status"

MOVE_TO_POINT_ACTION_NAME = "control/targets/move_to_point"


def topic_payload(topic_name, msg):
    """Build a websocket topic payload."""
    return {
        FIELD_TYPE: TYPE_TOPIC,
        FIELD_DATA: {
            FIELD_TOPIC_NAME: topic_name,
            FIELD_MSG: msg,
        },
    }
