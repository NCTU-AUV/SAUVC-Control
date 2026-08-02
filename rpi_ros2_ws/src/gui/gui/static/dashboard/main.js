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
