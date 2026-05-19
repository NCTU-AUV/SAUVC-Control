const protocol = GuiProtocol;
const websocket = new WebSocket(
    protocol.makeWebsocketUrl(window.location.hostname),
    protocol.websocketSubprotocol
);
const stm32LogState = {
    shouldAutoScroll: true,
};

function formatDisplayNumber(value, decimalPlaces) {
    let numberValue = null;
    if (typeof value === "number") {
        numberValue = value;
    } else if (typeof value === "string" && value.trim() !== "") {
        numberValue = Number(value);
    }
    if (Number.isFinite(numberValue)) {
        return numberValue.toFixed(decimalPlaces);
    }
    return value;
}

function isAtBottom(element, thresholdPx = 8) {
    return element.scrollTop + element.clientHeight >= element.scrollHeight - thresholdPx;
}

function updateStm32LogAutoScrollState() {
    const element = document.getElementById("stm32_debug_log");
    if (!element) {
        return;
    }
    stm32LogState.shouldAutoScroll = isAtBottom(element);
}

function formatOptionalDisplayNumber(value, decimalPlaces) {
    const formatted = formatDisplayNumber(value, decimalPlaces);
    return formatted ?? "none";
}

function setReadoutText(elementId, value, decimalPlaces = 3) {
    const element = document.getElementById(elementId);
    if (!element) {
        return;
    }
    if (!Number.isFinite(value)) {
        element.innerHTML = "none";
        return;
    }
    element.innerHTML = formatDisplayNumber(value, decimalPlaces);
}

function syncNumberInputIfIdle(elementId, value) {
    const element = document.getElementById(elementId);
    if (!element || document.activeElement === element || !Number.isFinite(value)) {
        return;
    }
    element.value = String(value);
}

function syncPidParamDisplay(prefix, params) {
    if (!params) {
        return;
    }

    const fieldMappings = [
        ["proportional_gain", "p"],
        ["integral_gain", "i"],
        ["derivative_gain", "d"],
        ["derivative_smoothing_factor", "smoothing"],
    ];

    fieldMappings.forEach(([paramName, fieldKey]) => {
        const value = Number(params[paramName]);
        setReadoutText(`${prefix}_${fieldKey}_current`, value, 3);
        syncNumberInputIfIdle(`${prefix}_${fieldKey}_input`, value);
    });
}

document.addEventListener("DOMContentLoaded", () => {
    const element = document.getElementById("stm32_debug_log");
    if (!element) {
        return;
    }
    element.addEventListener("scroll", updateStm32LogAutoScrollState);
    stm32LogState.shouldAutoScroll = isAtBottom(element);
});

websocket.onmessage = (event) => {
  console.log(event.data);

    const msg_json_object = JSON.parse(event.data);

    if (msg_json_object.type == protocol.types.topic) {
        if (msg_json_object.data.topic_name == protocol.topics.killed) {
            document.getElementById("killed").innerHTML = msg_json_object.data.msg;
        }
        if (msg_json_object.data.topic_name == protocol.topics.depthM) {
            document.getElementById("pressure_sensor_depth_m").innerHTML = formatDisplayNumber(
                msg_json_object.data.msg,
                3
            );
        }
        if (msg_json_object.data.topic_name == protocol.topics.targetDepthM) {
            const targetDepth = Number(msg_json_object.data.msg);
            setReadoutText("target_depth_m_current", targetDepth, 3);
            syncNumberInputIfIdle("target_depth_m_input", targetDepth);
        }
        if (msg_json_object.data.topic_name == protocol.topics.depthPidParams) {
            syncPidParamDisplay("depth_pid", msg_json_object.data.msg || {});
        }
        if (msg_json_object.data.topic_name == protocol.topics.bottomCameraPidParams) {
            const paramsByAxis = msg_json_object.data.msg || {};
            ["x", "y", "yaw"].forEach((axis) => {
                syncPidParamDisplay(`bottom_camera_${axis}_pid`, paramsByAxis[axis]);
            });
        }
        if (msg_json_object.data.topic_name == protocol.topics.thrustersPwmUs) {
            const pwmValues = msg_json_object.data.msg || [];
            for (let i = 0; i < 8; i += 1) {
                const value = pwmValues[i] ?? "";
                const element = document.getElementById("pwm_output_signal_value_us_" + i);
                if (element) {
                    element.innerHTML = value;
                }
            }
        }
        if (msg_json_object.data.topic_name == protocol.topics.thrustersEnabled) {
            const enabled = msg_json_object.data.msg === true;
            const element = document.getElementById("thrusters_enabled_status");
            if (element) {
                element.innerHTML = enabled ? "on" : "off";
            }
        }
        if (msg_json_object.data.topic_name == protocol.topics.systemManagerMode) {
            const mode = msg_json_object.data.msg;
            const element = document.getElementById("system_manager_mode");
            if (element) {
                element.innerHTML = mode;
            }
            const checkbox = document.getElementById("supervisor_manual_mode_input");
            if (checkbox) {
                checkbox.checked = mode === "MANUAL";
            }
        }
        if (msg_json_object.data.topic_name == protocol.topics.systemManagerStatus) {
            const element = document.getElementById("system_manager_status");
            if (element) {
                element.innerHTML = msg_json_object.data.msg;
            }
        }
        if (msg_json_object.data.topic_name == protocol.topics.electromagnetEnabled) {
            const enabled = msg_json_object.data.msg === true;
            const checkbox = document.getElementById("electromagnet_set_on_input");
            const status = document.getElementById("electromagnet_set_on_status");
            if (checkbox) {
                checkbox.checked = enabled;
            }
            if (status) {
                status.innerHTML = enabled ? "on" : "off";
            }
        }
        if (msg_json_object.data.topic_name == protocol.topics.flashStm32Status) {
            const status = msg_json_object.data.msg || {};
            const message = status.message || "";
            const successText = status.success === true ? "success" : "failed";
            const element = document.getElementById("flash_stm32_status");
            if (element) {
                element.innerHTML = message ? `${successText}: ${message}` : successText;
            }
        }
        if (msg_json_object.data.topic_name == protocol.topics.moveToPointStatus) {
            const status = msg_json_object.data.msg || {};
            const element = document.getElementById("move_to_point_status");
            if (element) {
                let text = status.state || "unknown";
                if (status.message) {
                    text += `: ${status.message}`;
                }
                if (typeof status.progress === "number") {
                    text += ` (${Math.round(status.progress * 100)}%)`;
                }
                if (typeof status.remaining_distance_px === "number") {
                    text += ` remaining ${status.remaining_distance_px.toFixed(1)} px`;
                }
                element.innerHTML = text;
            }
        }
        if (msg_json_object.data.topic_name == protocol.topics.bottomCameraPoseResetStatus) {
            const status = msg_json_object.data.msg || {};
            const element = document.getElementById("bottom_camera_pose_reset_status");
            if (element) {
                const successText = status.success === true ? "success" : "failed";
                element.innerHTML = status.message ? `${successText}: ${status.message}` : successText;
            }
        }
        if (msg_json_object.data.topic_name == protocol.topics.bottomCameraTopicStats) {
            const stats = msg_json_object.data.msg || {};
            const lkPoseHzElement = document.getElementById("bottom_camera_pose_hz");
            if (lkPoseHzElement) {
                lkPoseHzElement.innerHTML = formatOptionalDisplayNumber(stats.lk_pose_hz, 2);
            }
            const yawHzElement = document.getElementById("bottom_camera_yaw_hz");
            if (yawHzElement) {
                yawHzElement.innerHTML = formatOptionalDisplayNumber(stats.yaw_hz, 2);
            }
            const imageRawHzElement = document.getElementById("bottom_camera_image_raw_hz");
            if (imageRawHzElement) {
                imageRawHzElement.innerHTML = formatOptionalDisplayNumber(stats.image_raw_hz, 2);
            }
            const imageSizeElement = document.getElementById("bottom_camera_image_size");
            if (imageSizeElement) {
                const width = Number(stats.image_width);
                const height = Number(stats.image_height);
                if (Number.isFinite(width) && Number.isFinite(height)) {
                    imageSizeElement.innerHTML = `${width} x ${height}`;
                } else {
                    imageSizeElement.innerHTML = "none";
                }
            }
        }
        if (msg_json_object.data.topic_name == protocol.topics.stm32Log) {
            const element = document.getElementById("stm32_debug_log");
            if (element) {
                const line = msg_json_object.data.msg || "";
                if (line) {
                    const lines = element.textContent.split("\n").filter(Boolean);
                    lines.push(line);
                    const maxLines = 200;
                    const trimmed = lines.slice(-maxLines);
                    element.textContent = trimmed.join("\n");
                    if (stm32LogState.shouldAutoScroll) {
                        element.scrollTop = element.scrollHeight;
                    }
                }
            }
        }

        const bottomCameraPidTopicElements = {
            [protocol.topics.bottomCameraPidXReferencePx]: "bottom_camera_pid_x_reference_px",
            [protocol.topics.bottomCameraPidYReferencePx]: "bottom_camera_pid_y_reference_px",
            [protocol.topics.bottomCameraYawTargetRad]: "bottom_camera_yaw_target_rad",
            [protocol.topics.bottomCameraPidYawReferenceRad]: "bottom_camera_pid_yaw_reference_rad",
            [protocol.topics.bottomCameraPidXFeedbackPx]: "bottom_camera_pid_x_feedback_px",
            [protocol.topics.bottomCameraPidYFeedbackPx]: "bottom_camera_pid_y_feedback_px",
            [protocol.topics.bottomCameraPidYawFeedbackRad]: "bottom_camera_pid_yaw_feedback_rad",
        };
        const bottomCameraPidElementId = bottomCameraPidTopicElements[msg_json_object.data.topic_name];
        if (bottomCameraPidElementId) {
            const element = document.getElementById(bottomCameraPidElementId);
            if (element) {
                element.innerHTML = formatDisplayNumber(msg_json_object.data.msg, 2);
            }
        }
    }
};

websocket.onopen = (event) => {
    console.log("websocket.onopen");
};

function send_process_action(target, action) {
    websocket.send(JSON.stringify(protocol.makeProcessMessage(target, action)));
}

function send_controller_action(group, action) {
    websocket.send(JSON.stringify(protocol.makeControllerMessage(group, action)));
}

function enable_bottom_camera_pid_fbc() {
    send_controller_action(
        protocol.controllerGroups.bottomCameraPidFbc,
        protocol.controllerActions.enable
    );
}

function disable_bottom_camera_pid_fbc() {
    send_controller_action(
        protocol.controllerGroups.bottomCameraPidFbc,
        protocol.controllerActions.disable
    );
}

function reset_bottom_camera_pid_fbc() {
    send_controller_action(
        protocol.controllerGroups.bottomCameraPidFbc,
        protocol.controllerActions.reset
    );
}

function reset_bottom_camera_pose_button_onclick() {
    const element = document.getElementById("bottom_camera_pose_reset_status");
    if (element) {
        element.innerHTML = "resetting...";
    }
    websocket.send(JSON.stringify(protocol.makeActionMessage(
        protocol.actions.resetBottomCameraPose
    )));
}

function enable_depth_control() {
    send_controller_action(
        protocol.controllerGroups.depthControl,
        protocol.controllerActions.enable
    );
}

function disable_depth_control() {
    send_controller_action(
        protocol.controllerGroups.depthControl,
        protocol.controllerActions.disable
    );
}

function reset_depth_control() {
    send_controller_action(
        protocol.controllerGroups.depthControl,
        protocol.controllerActions.reset
    );
}

function set_supervisor_simulation_mode(enabled) {
    websocket.send(JSON.stringify(protocol.makeActionMessage(
        protocol.actions.setSupervisorSimulationMode,
        {enabled: enabled}
    )));
}

function set_supervisor_manual_mode(enabled) {
    websocket.send(JSON.stringify(protocol.makeActionMessage(
        protocol.actions.setSupervisorManualMode,
        {enabled: enabled}
    )));
}

function supervisor_manual_mode_input_onchange() {
    const checkbox = document.getElementById("supervisor_manual_mode_input");
    set_supervisor_manual_mode(Boolean(checkbox && checkbox.checked));
}

function supervisor_simulation_mode_input_onchange() {
    const checkbox = document.getElementById("supervisor_simulation_mode_input");
    set_supervisor_simulation_mode(Boolean(checkbox && checkbox.checked));
}

function set_target_depth_m_button_onclick() {
    const target_depth_m = document.getElementById("target_depth_m_input").value;
    websocket.send(JSON.stringify(protocol.makeTopicMessage(
        protocol.topics.targetDepthM,
        {data: target_depth_m}
    )));
}

function set_electromagnet_on(enabled) {
    websocket.send(JSON.stringify(protocol.makeTopicMessage(
        protocol.topics.electromagnetEnabled,
        {data: enabled}
    )));
}

function electromagnet_set_on_input_onchange() {
    const checkbox = document.getElementById("electromagnet_set_on_input");
    set_electromagnet_on(Boolean(checkbox && checkbox.checked));
}

function set_depth_pid_params_button_onclick() {
    const p = document.getElementById("depth_pid_p_input").value;
    const i = document.getElementById("depth_pid_i_input").value;
    const d = document.getElementById("depth_pid_d_input").value;
    const smoothing = document.getElementById("depth_pid_smoothing_input").value;

    websocket.send(JSON.stringify(protocol.makeControllerMessage(
        protocol.controllerGroups.depthControl,
        protocol.controllerActions.setPidParams,
        {
            params: {
                proportional_gain: p,
                integral_gain: i,
                derivative_gain: d,
                derivative_smoothing_factor: smoothing,
            }
        }
    )));
}

function set_bottom_camera_pid_params_button_onclick(axis) {
    const prefix = `bottom_camera_${axis}_pid`;
    const p = document.getElementById(`${prefix}_p_input`).value;
    const i = document.getElementById(`${prefix}_i_input`).value;
    const d = document.getElementById(`${prefix}_d_input`).value;
    const smoothing = document.getElementById(`${prefix}_smoothing_input`).value;

    websocket.send(JSON.stringify(protocol.makeControllerMessage(
        protocol.controllerGroups.bottomCameraPidFbc,
        protocol.controllerActions.setPidParams,
        {
            params: {
                axis: axis,
                proportional_gain: p,
                integral_gain: i,
                derivative_gain: d,
                derivative_smoothing_factor: smoothing,
            }
        }
    )));
}

function move_to_point_button_onclick() {
    const x_px = document.getElementById("move_to_point_x_px_input").value;
    const y_px = document.getElementById("move_to_point_y_px_input").value;
    const speed_px_s = document.getElementById("move_to_point_speed_px_s_input").value;
    const statusElement = document.getElementById("move_to_point_status");

    if (statusElement) {
        statusElement.innerHTML = "sending...";
    }

    websocket.send(JSON.stringify(protocol.makeActionMessage(
        protocol.actions.moveToPoint,
        {
            x_px: x_px,
            y_px: y_px,
            speed_px_s: speed_px_s,
        }
    )));
}

function publish_bottom_camera_yaw_target_rad(yaw_rad) {
    websocket.send(JSON.stringify(protocol.makeTopicMessage(
        protocol.topics.bottomCameraYawTargetRad,
        {data: yaw_rad}
    )));
}

function set_bottom_camera_yaw_target_rad_button_onclick() {
    const yaw_rad = document.getElementById("bottom_camera_yaw_target_rad_input").value;
    publish_bottom_camera_yaw_target_rad(yaw_rad);
}

function set_bottom_camera_yaw_target_deg_button_onclick() {
    const yaw_deg = document.getElementById("bottom_camera_yaw_target_deg_input").value;
    publish_bottom_camera_yaw_target_rad(Number(yaw_deg) * Math.PI / 180.0);
}

function cancel_move_to_point_button_onclick() {
    websocket.send(JSON.stringify(protocol.makeActionMessage(
        protocol.actions.cancelMoveToPoint
    )));
}

function initialize_all_thrusters_button_onclick(){
      console.log("initialize_all_thrusters_button_onclick");

      websocket.send(JSON.stringify(protocol.makeActionMessage(
          protocol.actions.initializeAllThrusters,
          {goal: ""}
      )));
}

function flash_stm32_button_onclick() {
    console.log("flash_stm32_button_onclick");
    const element = document.getElementById("flash_stm32_status");
    if (element) {
        element.innerHTML = "running...";
    }
    const logElement = document.getElementById("stm32_debug_log");
    if (logElement) {
        logElement.textContent = "";
        logElement.scrollTop = logElement.scrollHeight;
        stm32LogState.shouldAutoScroll = true;
    }
    websocket.send(JSON.stringify(protocol.makeActionMessage(protocol.actions.flashStm32)));
}

function clear_stm32_log_button_onclick() {
    const logElement = document.getElementById("stm32_debug_log");
    if (logElement) {
        logElement.textContent = "";
        logElement.scrollTop = logElement.scrollHeight;
        stm32LogState.shouldAutoScroll = true;
    }
}

function set_pwm_output_signal_value_us_button_onclick() {
    const pwm_values = [];
    for (let i = 0; i < 8; i += 1) {
        const value = document.getElementById("set_pwm_output_signal_value_us_" + i).value;
        pwm_values.push(value);
    }

    console.log("set_pwm_output_signal_value_us_button_onclick", pwm_values);

    websocket.send(JSON.stringify(protocol.makeTopicMessage(
        protocol.topics.thrustersPwmUs,
        {data: pwm_values}
    )));
}

function set_control_wrench_command_button_onclick() {
    var msg = {
        force: {
            x: document.getElementById("control_wrench_command_force_x").value,
            y: document.getElementById("control_wrench_command_force_y").value,
            z: document.getElementById("control_wrench_command_force_z").value,
        },
        torque: {
            x: document.getElementById("control_wrench_command_torque_x").value,
            y: document.getElementById("control_wrench_command_torque_y").value,
            z: document.getElementById("control_wrench_command_torque_z").value,
        }
    }

    console.log("set_control_wrench_command_button_onclick", msg);

    websocket.send(JSON.stringify(protocol.makeTopicMessage(
        protocol.topics.wrenchCommand,
        msg
    )));
}
