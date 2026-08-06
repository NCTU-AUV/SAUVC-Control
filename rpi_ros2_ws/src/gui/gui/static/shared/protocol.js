// Websocket protocol contract. Keep in step with backend/protocol.py —
// the two files are the same contract written twice, once per language.
const GuiProtocol = Object.freeze({
    websocketPath: "/websocket",
    websocketSubprotocol: "protocolOne",
    fields: Object.freeze({
        type: "type",
        data: "data",
        topicName: "topic_name",
        msg: "msg",
        actionName: "action_name",
        action: "action",
        group: "group",
        params: "params",
    }),
    types: Object.freeze({
        action: "action",
        topic: "topic",
        controller: "controller",
    }),
    actions: Object.freeze({
        initializeAllThrusters: "initialize_all_thrusters",
        flashStm32: "flash_stm32",
        setSupervisorSimulationMode: "set_supervisor_simulation_mode",
        setSupervisorManualMode: "set_supervisor_manual_mode",
        setSupervisorAutonomousMode: "set_supervisor_autonomous_mode",
        setSupervisorDepthHold: "set_supervisor_depth_hold",
        safeDisable: "safe_disable",
        startMission: "start_mission",
    }),
    controllerGroups: Object.freeze({
        depthControl: "depth_control",
    }),
    controllerActions: Object.freeze({
        enable: "enable",
        disable: "disable",
        reset: "reset",
        setPidParams: "set_pid_params",
    }),
    topics: Object.freeze({
        killed: "sensors/killed",
        depthM: "sensors/depth_m",
        stm32Log: "diagnostics/stm32/log",
        systemManagerMode: "system_manager/mode",
        systemManagerStatus: "system_manager/status",
        thrustersPwmUs: "thrusters/pwm_us",
        thrustersEnabled: "thrusters/enabled",
        electromagnetEnabled: "actuators/electromagnet/enabled",
        wrenchCommand: "control/wrench_command",
        targetDepthM: "control/targets/depth_m",
        depthPidParams: "control/pid/depth/gui_params",
        flashStm32Status: "flash_stm32_status",
        // GUI-only channels — not real ROS topics.
        serviceResult: "gui/service_result",
        bagStatus: "gui/bag_status",
        cameraSources: "gui/camera_sources",
    }),
    // Vehicle modes as reported by system_manager/mode.
    modes: Object.freeze({
        safeDisabled: "SAFE_DISABLED",
        manual: "MANUAL",
        depthHold: "DEPTH_HOLD",
        autonomous: "AUTONOMOUS",
        autonomousAndDepthHold: "AUTONOMOUS_AND_DEPTH_HOLD",
        fault: "FAULT",
    }),
    makeWebsocketUrl(hostname) {
        return "ws://" + hostname + this.websocketPath;
    },
    makeTopicMessage(topicName, msg) {
        return {
            type: this.types.topic,
            data: {
                topic_name: topicName,
                msg: msg,
            },
        };
    },
    makeActionMessage(actionName, data = {}) {
        return {
            type: this.types.action,
            data: {
                ...data,
                action_name: actionName,
            },
        };
    },
    makeControllerMessage(group, action, data = {}) {
        return {
            type: this.types.controller,
            data: {
                ...data,
                group: group,
                action: action,
            },
        };
    },
});

if (typeof window !== "undefined") {
    window.GuiProtocol = GuiProtocol;
}
