const wsStatusElement = document.getElementById("ws_status");
const gamepadStatusElement = document.getElementById("gamepad_status");
const thrustXElement = document.getElementById("thrust_x");
const thrustYElement = document.getElementById("thrust_y");
const thrustZElement = document.getElementById("thrust_z");
const thrustYawElement = document.getElementById("thrust_yaw");
const cameraFeedElement = document.getElementById("camera_feed");
const thrusterPwmElements = Array.from({ length: 8 }, (_, index) =>
    document.getElementById(`thruster_pwm_${index}`)
);
const protocol = GuiProtocol;

const max_push = 10;
const max_twist = 5;
const deadzone = 0.15;
const sendIntervalMs = 50; // 20 Hz

const current_wrench = {
    force: {
        x: 0,
        y: 0,
        z: 0,
    },
    torque: {
        x: 0,
        y: 0,
        z: 0,
    },
};

let activeGamepadIndex = null;
let animationFrameId = null;
let lastSendTime = 0;
let lastBPressed = false;
const pressedKeys = new Set();

const websocket = new WebSocket(
    protocol.makeWebsocketUrl(window.location.hostname),
    protocol.websocketSubprotocol
);

function initializeCameraFeed() {
    if (!cameraFeedElement) {
        return;
    }

    // 影像來源。底部相機退場後，這裡預期接的是 Autonomy 堆疊的 RealSense。
    // web_video_server 會自己去 graph 上找這個 topic，找不到就顯示不出來，
    // 不影響其他控制功能。
    const cameraTopic = window.ORCA_CAMERA_TOPIC || "/orca/color/image_raw";
    const streamUrl = "http://" + window.location.hostname + ":8080/stream?topic="
        + cameraTopic
        + "&type=ros_compressed";
    cameraFeedElement.src = streamUrl;
}

function updateWsStatus(connected) {
    if (!wsStatusElement) {
        return;
    }
    wsStatusElement.textContent = connected ? "🟢 Connected" : "🔴 Disconnected";
}

function updateGamepadStatus(text) {
    if (!gamepadStatusElement) {
        return;
    }
    gamepadStatusElement.textContent = text;
}

function updateHudWrench() {
    if (thrustXElement) {
        thrustXElement.textContent = current_wrench.force.x.toFixed(2);
    }
    if (thrustYElement) {
        thrustYElement.textContent = current_wrench.force.y.toFixed(2);
    }
    if (thrustZElement) {
        thrustZElement.textContent = current_wrench.force.z.toFixed(2);
    }
    if (thrustYawElement) {
        thrustYawElement.textContent = current_wrench.torque.z.toFixed(2);
    }
}

function applyDeadzone(value) {
    if (Math.abs(value) < deadzone) {
        return 0;
    }
    return value;
}

function buildWrenchPayload() {
    return JSON.stringify(protocol.makeTopicMessage(
        protocol.topics.wrenchCommand,
        current_wrench
    ));
}

function sendCurrentWrench() {
    if (websocket.readyState !== WebSocket.OPEN) {
        return;
    }
    websocket.send(buildWrenchPayload());
}

function setWrenchToZero() {
    current_wrench.force.x = 0;
    current_wrench.force.y = 0;
    current_wrench.force.z = 0;
    current_wrench.torque.x = 0;
    current_wrench.torque.y = 0;
    current_wrench.torque.z = 0;
    updateHudWrench();
}

function updateThrusterPwmDisplay(pwmValues) {
    for (let index = 0; index < thrusterPwmElements.length; index += 1) {
        const element = thrusterPwmElements[index];
        if (!element) {
            continue;
        }
        const value = pwmValues[index];
        element.textContent = Number.isFinite(value) ? String(Math.round(value)) : "--";
    }
}

function findFirstConnectedGamepad() {
    if (typeof navigator.getGamepads !== "function") {
        return null;
    }
    const gamepads = navigator.getGamepads();
    for (let i = 0; i < gamepads.length; i += 1) {
        if (gamepads[i]) {
            return gamepads[i];
        }
    }
    return null;
}

function getActiveGamepad() {
    const gamepads = navigator.getGamepads();
    if (activeGamepadIndex !== null) {
        const selected = gamepads[activeGamepadIndex] || null;
        if (selected) {
            return selected;
        }
        activeGamepadIndex = null;
        lastBPressed = false;
    }

    const firstConnected = findFirstConnectedGamepad();
    if (firstConnected) {
        activeGamepadIndex = firstConnected.index;
        return firstConnected;
    }

    return null;
}

function applyGamepadInput(gamepad) {
    updateGamepadStatus("🎮 已連接: " + gamepad.id);

    const leftX = applyDeadzone(gamepad.axes[0] || 0);
    const leftY = applyDeadzone(gamepad.axes[1] || 0);
    const rightX = applyDeadzone(gamepad.axes[2] || 0);
    const rightY = applyDeadzone(gamepad.axes[3] || 0);

    // Left stick up/down controls surge on force.x (up -> positive x)
    current_wrench.force.x = -leftY * max_push;
    // Left stick left/right controls sway on force.y
    current_wrench.force.y = leftX * max_push;
    // Right stick up/down controls heave on force.z
    current_wrench.force.z = -rightY * max_push;
    // Right stick left/right controls yaw on torque.z
    current_wrench.torque.z = rightX * max_twist;

    updateHudWrench();

    const bPressed = Boolean(gamepad.buttons[1] && gamepad.buttons[1].pressed);
    if (bPressed && !lastBPressed) {
        setWrenchToZero();
        sendCurrentWrench();
    }
    lastBPressed = bPressed;
}

function applyKeyboardInput() {
    const forward = pressedKeys.has("w");
    const backward = pressedKeys.has("s");
    const left = pressedKeys.has("a");
    const right = pressedKeys.has("d");
    const up = pressedKeys.has("r") || pressedKeys.has("arrowup");
    const down = pressedKeys.has("f") || pressedKeys.has("arrowdown");
    const yawLeft = pressedKeys.has("q") || pressedKeys.has("arrowleft");
    const yawRight = pressedKeys.has("e") || pressedKeys.has("arrowright");

    current_wrench.force.x = (forward ? max_push : 0) + (backward ? -max_push : 0);
    current_wrench.force.y = (right ? max_push : 0) + (left ? -max_push : 0);
    current_wrench.force.z = (up ? max_push : 0) + (down ? -max_push : 0);
    current_wrench.torque.z = (yawRight ? max_twist : 0) + (yawLeft ? -max_twist : 0);

    if (pressedKeys.size > 0) {
        updateGamepadStatus("⌨️ 鍵盤控制中");
    } else {
        updateGamepadStatus("🎮 未連接，使用鍵盤控制");
    }

    updateHudWrench();
}

function tickControls() {
    const gamepad = getActiveGamepad();
    if (gamepad) {
        applyGamepadInput(gamepad);
    } else {
        lastBPressed = false;
        applyKeyboardInput();
    }

    const now = performance.now();
    if (now - lastSendTime >= sendIntervalMs) {
        sendCurrentWrench();
        lastSendTime = now;
    }

    animationFrameId = requestAnimationFrame(tickControls);
}

window.addEventListener("gamepadconnected", (event) => {
    activeGamepadIndex = event.gamepad.index;
    updateGamepadStatus("🎮 已連接: " + event.gamepad.id);
});

window.addEventListener("gamepaddisconnected", (event) => {
    if (activeGamepadIndex === event.gamepad.index) {
        activeGamepadIndex = null;
        lastBPressed = false;
        updateGamepadStatus("🎮 未連接，使用鍵盤控制");
        setWrenchToZero();
        sendCurrentWrench();
    }
});

window.addEventListener("keydown", (event) => {
    const key = event.key.toLowerCase();
    const controlKeys = new Set([
        "w", "a", "s", "d", "q", "e", "r", "f",
        "arrowup", "arrowdown", "arrowleft", "arrowright",
    ]);
    if (!controlKeys.has(key)) {
        return;
    }
    event.preventDefault();
    pressedKeys.add(key);
});

window.addEventListener("keyup", (event) => {
    const key = event.key.toLowerCase();
    if (pressedKeys.delete(key)) {
        event.preventDefault();
    }
});

window.addEventListener("blur", () => {
    pressedKeys.clear();
    setWrenchToZero();
    sendCurrentWrench();
});

websocket.onopen = () => {
    updateWsStatus(true);
};

websocket.onmessage = (event) => {
    let msgJsonObject;
    try {
        msgJsonObject = JSON.parse(event.data);
    } catch (_error) {
        return;
    }

    if (msgJsonObject.type !== protocol.types.topic) {
        return;
    }

    const topicName = msgJsonObject.data && msgJsonObject.data.topic_name;
    const topicMsg = msgJsonObject.data && msgJsonObject.data.msg;
    if (topicName !== protocol.topics.thrustersPwmUs || !Array.isArray(topicMsg)) {
        return;
    }

    const normalizedValues = topicMsg.slice(0, 8).map((value) => Number(value));
    updateThrusterPwmDisplay(normalizedValues);
};

websocket.onclose = () => {
    updateWsStatus(false);
};

websocket.onerror = () => {
    updateWsStatus(false);
};

updateWsStatus(false);
updateHudWrench();
updateThrusterPwmDisplay(Array(8).fill(1500));
initializeCameraFeed();
animationFrameId = requestAnimationFrame(tickControls);
