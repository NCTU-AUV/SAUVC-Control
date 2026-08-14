// Dashboard wiring. Layout is index.html, tokens are shared/theme.css,
// transport is shared/ws.js.

const protocol = window.GuiProtocol;
const socket = new window.GuiSocket(protocol);

const $ = (id) => document.getElementById(id);

// How long a topic may go quiet before its readout is marked stale. Generous
// enough not to flicker on a slow tick, tight enough that a dead link shows up
// before somebody acts on a frozen number.
const STALE_MS = {
    [protocol.topics.depthM]: 1500,
    [protocol.topics.targetDepthM]: 8000,
    [protocol.topics.killed]: 8000,
    [protocol.topics.thrustersEnabled]: 8000,
    [protocol.topics.systemManagerMode]: 8000,
};

const state = {
    mode: null,
    targetDepth: 0,
    killed: null,
    cameras: [],
    cameraPort: 8080,
    mainCameraId: null,
};

// ---------------------------------------------------------------- formatting

function fmt(value, places = 2) {
    const n = Number(value);
    return Number.isFinite(n) ? n.toFixed(places) : "—";
}

function setMetric(id, value, places, unit = "") {
    const el = $(id);
    if (!el) return;
    const text = fmt(value, places);
    el.innerHTML = unit
        ? `${text}<span class="readout-unit">${unit}</span>`
        : text;
}

function setPill(id, text, tone) {
    const el = $(id);
    if (!el) return;
    el.textContent = text;
    el.className = "pill" + (tone ? ` ${tone}` : "");
}

// ------------------------------------------------------------------- toasts

function toast(title, body, tone = "") {
    const stack = $("toast_stack");
    const el = document.createElement("div");
    el.className = "toast" + (tone ? ` ${tone}` : "");
    el.innerHTML = `<div class="toast-title"></div><div class="toast-body"></div>`;
    el.querySelector(".toast-title").textContent = title;
    el.querySelector(".toast-body").textContent = body || "";
    stack.appendChild(el);
    // Failures stay long enough to read and copy; successes get out of the way.
    window.setTimeout(() => el.remove(), tone === "crit" ? 9000 : 4000);
}

// ------------------------------------------------------------------- confirm

let confirmResolve = null;

function confirmAction(title, body) {
    $("confirm_title").textContent = title;
    $("confirm_body").textContent = body;
    $("confirm_modal").hidden = false;
    $("confirm_ok").focus();
    return new Promise((resolve) => { confirmResolve = resolve; });
}

function closeConfirm(result) {
    $("confirm_modal").hidden = true;
    if (confirmResolve) {
        confirmResolve(result);
        confirmResolve = null;
    }
}

$("confirm_ok").addEventListener("click", () => closeConfirm(true));
$("confirm_cancel").addEventListener("click", () => closeConfirm(false));
$("confirm_modal").addEventListener("click", (e) => {
    if (e.target === $("confirm_modal")) closeConfirm(false);
});

// ---------------------------------------------------------------------- tabs

document.querySelectorAll(".tab").forEach((tab) => {
    tab.addEventListener("click", () => {
        document.querySelectorAll(".tab").forEach((t) => {
            const selected = t === tab;
            t.setAttribute("aria-selected", String(selected));
            $(`tab_${t.dataset.tab}`).hidden = !selected;
        });
    });
});

// ---------------------------------------------------------------------- mode
//
// Every toggle is derived from system_manager/mode, never from what the
// operator last clicked. The old dashboard only synced the manual checkbox, so
// after the vehicle faulted on its own the autonomy box still showed enabled —
// the UI and the vehicle disagreed and only the vehicle was right.

const MODE_TONE = {
    [protocol.modes.fault]: "crit",
    [protocol.modes.safeDisabled]: "",
    [protocol.modes.manual]: "warn",
    [protocol.modes.depthHold]: "ok",
    [protocol.modes.autonomous]: "ok",
    [protocol.modes.autonomousAndDepthHold]: "ok",
};

function applyMode(mode) {
    state.mode = mode;
    const faulted = mode === protocol.modes.fault;

    setPill("mode_pill", mode || "—", MODE_TONE[mode] ?? "");

    const depthHold = mode === protocol.modes.depthHold
        || mode === protocol.modes.autonomousAndDepthHold;
    const autonomous = mode === protocol.modes.autonomous
        || mode === protocol.modes.autonomousAndDepthHold;
    const manual = mode === protocol.modes.manual;

    $("toggle_depth_hold").setAttribute("aria-pressed", String(depthHold));
    $("toggle_autonomous").setAttribute("aria-pressed", String(autonomous));
    $("toggle_manual").setAttribute("aria-pressed", String(manual));

    // In FAULT the supervisor rejects every enable request, so present them as
    // unavailable rather than letting the operator click into a refusal.
    for (const id of ["toggle_depth_hold", "toggle_autonomous", "toggle_manual"]) {
        $(id).setAttribute("aria-disabled", String(faulted));
    }
    $("mode_hint").textContent = faulted
        ? "Latched in FAULT. Press STOP to clear it before arming."
        : "";

    updateKeyboardAvailability();
}

// ------------------------------------------------------------------ keyboard

function depthHoldActive() {
    return state.mode === protocol.modes.depthHold
        || state.mode === protocol.modes.autonomousAndDepthHold;
}

// Keys are live in the two modes where the operator is the one flying.
// wrench_sum has no per-mode source filter — a single global active flag — so
// the GUI wrench really is summed in DEPTH_HOLD too, verified on the running
// stack: force.x 20 arrived at control/wrench_command next to the PID's
// force.z. Deliberately not enabled in the autonomous modes, where piloting
// would silently fight the decision node on the same bus.
function pilotingAllowed() {
    return state.mode === protocol.modes.manual || depthHoldActive();
}

const pilot = new window.KeyboardPilot(socket, protocol, {
    getEnabled: pilotingAllowed,
    getTargetDepth: () => state.targetDepth,
    onState: (s) => {
        document.querySelectorAll(".key").forEach((el) => {
            el.classList.toggle("held", s.held.has(el.dataset.key));
        });
    },
});
pilot.start();

function updateKeyboardAvailability() {
    const manual = state.mode === protocol.modes.manual;
    const holding = depthHoldActive();
    const active = pilotingAllowed();
    setPill("keyboard_pill", active ? "active" : "inactive", active ? "ok" : "");

    if (holding) {
        $("keyboard_hint").textContent =
            "WASD steers, Q/E moves the depth target and the PID flies to it.";
        $("keyboard_hint").className = "hint";
    } else if (manual) {
        // Worth saying plainly: MANUAL is the one mode that deactivates the
        // depth PID, so the setpoint Q/E writes has nothing acting on it.
        $("keyboard_hint").textContent =
            "WASD steers. Q/E still moves the depth target, but Manual "
            + "deactivates the depth PID so nothing acts on it — switch to "
            + "Depth hold to actually change depth.";
        $("keyboard_hint").className = "hint warn";
    } else {
        $("keyboard_hint").textContent =
            "Enable Depth hold (or Manual) to pilot from the keyboard.";
        $("keyboard_hint").className = "hint";
    }
    $("kb_vertical_note").textContent = holding
        ? "depth target" : manual ? "depth target (PID off)" : "unavailable";
}

$("kb_force_input").addEventListener("change", (e) => {
    pilot.translationForceN = Number(e.target.value) || 0;
});
$("kb_torque_input").addEventListener("change", (e) => {
    pilot.yawTorqueNm = Number(e.target.value) || 0;
});
$("kb_depth_step_input").addEventListener("change", (e) => {
    pilot.depthStepM = Number(e.target.value) || 0;
});

// -------------------------------------------------------------------- camera

// The topic goes in raw. web_video_server does not URL-decode the query
// parameter, so percent-encoding the slashes makes it reject the request with
// "Invalid topic name" and return an empty multipart body — a stream that
// connects and then shows nothing. Topic names only ever contain slashes,
// alphanumerics and underscores, all of which are safe here unescaped.
function cameraUrl(source, endpoint) {
    const extra = source.params ? `&${source.params}` : "";
    return `http://${window.location.hostname}:${state.cameraPort}`
        + `/${endpoint}?topic=${source.topic}${extra}`;
}

// Only the main view holds a live MJPEG connection. Thumbnails poll /snapshot
// once a second instead.
//
// Four simultaneous MJPEG streams do not survive: they are four long-lived
// connections, and in practice only two ever delivered frames — the rest sat
// at 200 OK with nothing arriving, for 40 s and counting. Polling also costs a
// fraction of the bandwidth, which matters on a laptop at the poolside, and a
// thumbnail exists to answer "which channel do I want to look at", for which
// 1 Hz is plenty.
const THUMB_REFRESH_MS = 1000;
let thumbTimer = null;

function refreshThumbs() {
    for (const img of document.querySelectorAll(".camera-thumb img")) {
        const base = img.dataset.base;
        if (base) {
            img.src = `${base}&_=${Date.now()}`;
        }
    }
}

// Availability comes from the backend, which checks the ROS graph for a
// publisher. The browser cannot work it out for itself: an <img> pointed at an
// MJPEG stream fires neither load nor error dependably, because the response is
// an endless multipart body rather than one image.
function renderCameras() {
    if (!state.cameras.length) return;
    if (!state.cameras.some((c) => c.id === state.mainCameraId)) {
        state.mainCameraId = state.cameras[0].id;
    }

    const main = state.cameras.find((c) => c.id === state.mainCameraId);
    const mainImg = $("camera_main_img");
    $("camera_main_label").textContent = main.label;
    $("camera_main_empty").hidden = main.available;
    const mainUrl = main.available ? cameraUrl(main, "stream") : "";
    // Only reassign when it actually changed, or the stream restarts on every
    // refresh and the picture visibly stutters.
    if (mainImg.getAttribute("src") !== mainUrl) {
        mainImg.src = mainUrl;
    }
    mainImg.hidden = !main.available;

    const thumbs = $("camera_thumbs");
    const wanted = state.cameras.filter((c) => c.id !== main.id);
    const signature = wanted
        .map((c) => `${c.id}:${c.topic}:${c.available}:${c.params}`).join("|");
    if (thumbs.dataset.signature === signature) return;
    thumbs.dataset.signature = signature;
    thumbs.innerHTML = "";

    for (const source of wanted) {
        const button = document.createElement("button");
        button.className = "camera-thumb";
        button.title = `Show ${source.label}`;
        if (source.available) {
            const img = document.createElement("img");
            img.alt = source.label;
            img.dataset.base = cameraUrl(source, "snapshot");
            img.src = img.dataset.base;
            button.appendChild(img);
        } else {
            const empty = document.createElement("div");
            empty.className = "camera-empty";
            empty.textContent = "No stream";
            button.appendChild(empty);
        }
        const badge = document.createElement("div");
        badge.className = "camera-badge";
        badge.textContent = source.label;
        button.appendChild(badge);
        button.addEventListener("click", () => {
            state.mainCameraId = source.id;
            renderCameras();
        });
        thumbs.appendChild(button);
    }

    if (thumbTimer === null) {
        thumbTimer = window.setInterval(refreshThumbs, THUMB_REFRESH_MS);
    }
}

// -------------------------------------------------------------- thruster grid

const THRUSTER_ORDER = [4, 5, 0, 1, 2, 3, 6, 7];

function buildThrusterGrid() {
    const grid = $("thruster_grid");
    const image = document.createElement("div");
    image.className = "thruster-img";
    image.innerHTML =
        '<img src="/static/shared/thruster_numbering.png" alt="Thruster numbering">';
    grid.appendChild(image);

    for (const index of THRUSTER_ORDER) {
        const row = document.createElement("div");
        row.className = `thruster-row t${index}`;
        row.innerHTML = `
            <span class="label">T${index}</span>
            <input type="number" min="0" max="3000" value="1500"
                   id="pwm_set_${index}" aria-label="Thruster ${index} PWM">
            <span class="num" id="pwm_rx_${index}">—</span>`;
        grid.appendChild(row);
    }
}
buildThrusterGrid();

// --------------------------------------------------------------- topic wiring

socket.onTopic(protocol.topics.systemManagerMode, applyMode);

socket.onTopic(protocol.topics.systemManagerStatus, (msg) => {
    // The supervisor puts the fault reason here; it is the only record of why
    // the vehicle stopped, so it stays on screen rather than in a toast.
    $("fault_reason").textContent =
        state.mode === protocol.modes.fault ? String(msg ?? "") : "";
});

socket.onTopic(protocol.topics.depthM, (msg) => {
    setMetric("depth_value", msg, 2, "m");
});

socket.onTopic(protocol.topics.targetDepthM, (msg) => {
    state.targetDepth = Number(msg);
    setMetric("target_depth_value", msg, 2, "m");
    const input = $("target_depth_input");
    if (document.activeElement !== input && Number.isFinite(state.targetDepth)) {
        input.value = state.targetDepth.toFixed(2);
    }
});

socket.onTopic(protocol.topics.killed, (msg) => {
    state.killed = msg === true;
    setPill("kill_pill", state.killed ? "Kill ACTIVE" : "Kill clear",
        state.killed ? "crit" : "ok");
});

socket.onTopic(protocol.topics.thrustersEnabled, (msg) => {
    setPill("thrusters_pill", msg === true ? "Thrusters on" : "Thrusters off",
        msg === true ? "ok" : "");
});

socket.onTopic(protocol.topics.electromagnetEnabled, (msg) => {
    setPill("magnet_pill", msg === true ? "holding" : "released",
        msg === true ? "ok" : "");
});

socket.onTopic(protocol.topics.thrustersPwmUs, (values) => {
    const list = values || [];
    for (let i = 0; i < 8; i += 1) {
        const el = $(`pwm_rx_${i}`);
        if (el) el.textContent = list[i] ?? "—";
    }
});

socket.onTopic(protocol.topics.depthPidParams, (params) => {
    const map = {
        proportional_gain: "p",
        integral_gain: "i",
        derivative_gain: "d",
        derivative_smoothing_factor: "smoothing",
    };
    for (const [key, suffix] of Object.entries(map)) {
        const value = Number((params || {})[key]);
        const readout = $(`pid_${suffix}_current`);
        if (readout) readout.textContent = fmt(value, 3);
        const input = $(`pid_${suffix}_input`);
        if (input && document.activeElement !== input && Number.isFinite(value)) {
            input.value = String(value);
        }
    }
});

socket.onTopic(protocol.topics.serviceResult, (result) => {
    const ok = result?.success === true;
    toast(ok ? "Accepted" : "Rejected",
        `${result?.service ?? ""} — ${result?.message ?? ""}`,
        ok ? "ok" : "crit");
});

socket.onTopic(protocol.topics.bagStatus, (status) => {
    const recording = status?.recording === true;
    setPill("bag_pill", recording ? "Bag REC" : "Bag idle", recording ? "ok" : "");
    $("bag_state").textContent = recording ? "recording" : "not running";
    $("bag_name").textContent = status?.bag_name ?? "—";
    $("bag_size").textContent =
        status?.size_mb == null ? "—" : `${status.size_mb} MB`;
    $("bag_free").textContent =
        status?.free_gb == null ? "—" : `${status.free_gb} GB`;
});

socket.onTopic(protocol.topics.cameraSources, (payload) => {
    state.cameraPort = payload?.port ?? 8080;
    state.cameras = payload?.sources ?? [];
    renderCameras();
});

socket.onTopic(protocol.topics.flashStm32Status, (status) => {
    const ok = status?.success === true;
    $("flash_status").textContent =
        `${ok ? "success" : "failed"}${status?.message ? `: ${status.message}` : ""}`;
});

socket.onTopic(protocol.topics.stm32Log, (line) => {
    if (!line) return;
    const el = $("stm32_log");
    const atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 8;
    const lines = el.textContent.split("\n").filter(Boolean);
    lines.push(String(line));
    el.textContent = lines.slice(-200).join("\n");
    if (atBottom) el.scrollTop = el.scrollHeight;
});

socket.onConnectionChange((connected) => {
    setPill("link_pill", connected ? "Link up" : "Link down",
        connected ? "ok" : "crit");
    if (!connected) {
        // Values on screen are now history. Say so rather than letting them
        // sit there looking current.
        applyMode(null);
    }
});

// ------------------------------------------------------------------- staleness

window.setInterval(() => {
    const marks = [
        ["depth_value", protocol.topics.depthM],
        ["target_depth_value", protocol.topics.targetDepthM],
    ];
    for (const [id, topic] of marks) {
        $(id)?.classList.toggle("is-stale", socket.isStale(topic, STALE_MS[topic]));
    }
}, 500);

// --------------------------------------------------------------------- actions

function toggleHandler(id, action) {
    $(id).addEventListener("click", () => {
        const el = $(id);
        if (el.getAttribute("aria-disabled") === "true") return;
        const next = el.getAttribute("aria-pressed") !== "true";
        socket.sendAction(action, {enabled: next});
        // Deliberately not flipping the pill here: it moves only when
        // system_manager/mode says the vehicle actually changed.
    });
}

toggleHandler("toggle_depth_hold", protocol.actions.setSupervisorDepthHold);
toggleHandler("toggle_autonomous", protocol.actions.setSupervisorAutonomousMode);
toggleHandler("toggle_manual", protocol.actions.setSupervisorManualMode);
toggleHandler("toggle_simulation", protocol.actions.setSupervisorSimulationMode);

// Stop the tree as well as the bus. gui_node also does this off the mode
// topic, but sending it here means the mission stops on the same click rather
// than one supervisor round-trip later.
function emergencyStop() {
    socket.sendAction(protocol.actions.stopMission);
    socket.sendAction(protocol.actions.safeDisable);
}

$("estop_button").addEventListener("click", emergencyStop);

// Esc is the stop key. No confirmation: a stop that needs a second click is
// not an emergency stop.
window.addEventListener("keydown", (e) => {
    if (e.key !== "Escape") return;
    if (!$("confirm_modal").hidden) { closeConfirm(false); return; }
    emergencyStop();
});

$("button_reset_controllers").addEventListener("click", () => {
    socket.sendController(protocol.controllerGroups.depthControl,
        protocol.controllerActions.reset);
});

$("button_set_depth").addEventListener("click", () => {
    socket.sendTopic(protocol.topics.targetDepthM,
        {data: $("target_depth_input").value});
});

$("button_start_mission").addEventListener("click", () => {
    socket.sendAction(protocol.actions.startMission);
});

$("button_stop_mission").addEventListener("click", () => {
    socket.sendAction(protocol.actions.stopMission);
});

$("button_magnet_on").addEventListener("click", () => {
    socket.sendTopic(protocol.topics.electromagnetEnabled, {data: true});
});
$("button_magnet_off").addEventListener("click", () => {
    socket.sendTopic(protocol.topics.electromagnetEnabled, {data: false});
});

$("button_set_pid").addEventListener("click", () => {
    socket.sendController(
        protocol.controllerGroups.depthControl,
        protocol.controllerActions.setPidParams,
        {
            params: {
                proportional_gain: $("pid_p_input").value,
                integral_gain: $("pid_i_input").value,
                derivative_gain: $("pid_d_input").value,
                derivative_smoothing_factor: $("pid_smoothing_input").value,
            },
        });
});

$("button_send_wrench").addEventListener("click", () => {
    socket.sendTopic(protocol.topics.wrenchCommand, {
        force: {
            x: $("wrench_fx").value,
            y: $("wrench_fy").value,
            z: $("wrench_fz").value,
        },
        torque: {
            x: $("wrench_tx").value,
            y: $("wrench_ty").value,
            z: $("wrench_tz").value,
        },
    });
});

// --- destructive, behind a confirmation -------------------------------------

$("button_send_pwm").addEventListener("click", async () => {
    const values = [];
    for (let i = 0; i < 8; i += 1) values.push($(`pwm_set_${i}`).value);
    const ok = await confirmAction(
        "Publish raw PWM?",
        "This writes the thrusters directly, bypassing the wrench bus and the "
        + "supervisor mode gate. The vehicle will move even in SAFE_DISABLED. "
        + `Values: ${values.join(", ")}`);
    if (ok) socket.sendTopic(protocol.topics.thrustersPwmUs, {data: values});
});

$("button_init_thrusters").addEventListener("click", async () => {
    const ok = await confirmAction(
        "Initialise all thrusters?",
        "Runs the ESC arming sequence. Keep hands and tools clear of the props.");
    if (ok) socket.sendAction(protocol.actions.initializeAllThrusters);
});

$("button_flash").addEventListener("click", async () => {
    const ok = await confirmAction(
        "Flash STM32 firmware?",
        "Overwrites the firmware on the microcontroller. The vehicle will be "
        + "uncontrollable until it finishes and reboots.");
    if (!ok) return;
    $("flash_status").textContent = "running…";
    $("stm32_log").textContent = "";
    socket.sendAction(protocol.actions.flashStm32);
});

$("button_clear_log").addEventListener("click", () => {
    $("stm32_log").textContent = "";
});

// ----------------------------------------------------------------------- go

// Connect even if the initial paint throws. An exception on the way down this
// file used to take the websocket with it: the page rendered, every readout sat
// at its placeholder, and the only clue was "Link down" — which reads as a
// server problem rather than a bug three lines earlier in the browser.
try {
    applyMode(null);
} catch (error) {
    console.error("gui: initial render failed", error);
    toast("GUI error", String(error && error.message || error), "crit");
}

socket.connect();
