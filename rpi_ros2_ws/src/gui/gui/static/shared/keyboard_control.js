// Keyboard piloting: WASD translates, Q/E steps the depth setpoint.
//
// Two mechanisms on purpose, because they are genuinely different things:
//
//   WASD publishes an open-loop wrench onto control/wrench_sources/gui, which
//   the supervisor only lets through in MANUAL. Held keys are composed every
//   tick, so W+D is a diagonal rather than whichever key won a race.
//
//   Q/E moves control/targets/depth_m and lets the depth PID hold it. Stepping
//   a setpoint is a discrete event, so those fire on keydown and repeat on a
//   slower timer than the wrench.

const WRENCH_HZ = 20;
const DEPTH_REPEAT_MS = 250;

const AXES = Object.freeze({
    KeyW: {axis: "surge", sign: +1},
    KeyS: {axis: "surge", sign: -1},
    KeyD: {axis: "sway", sign: +1},
    KeyA: {axis: "sway", sign: -1},
    ArrowRight: {axis: "yaw", sign: +1},
    ArrowLeft: {axis: "yaw", sign: -1},
});

const DEPTH_KEYS = Object.freeze({
    KeyE: +1,   // deeper: down is positive in this stack
    KeyQ: -1,   // shallower
});

class KeyboardPilot {
    /**
     * @param socket       GuiSocket
     * @param protocol     GuiProtocol
     * @param options      {getEnabled, getTargetDepth, onState}
     *   getEnabled     () => bool   — gate; keys are ignored when false
     *   getTargetDepth () => number — current setpoint, for Q/E stepping
     *   onState        (state) => void — for the on-screen key map
     */
    constructor(socket, protocol, options = {}) {
        this.socket = socket;
        this.protocol = protocol;
        this.getEnabled = options.getEnabled || (() => true);
        this.getTargetDepth = options.getTargetDepth || (() => 0);
        this.onState = options.onState || (() => {});

        this.translationForceN = 20;
        this.yawTorqueNm = 10;
        this.depthStepM = 0.05;

        this.held = new Set();
        this._wrenchTimer = null;
        this._depthTimer = null;
        this._wasPublishing = false;
    }

    start() {
        window.addEventListener("keydown", this._onKeyDown);
        window.addEventListener("keyup", this._onKeyUp);
        // Losing focus must release everything. Without this, alt-tabbing while
        // holding W leaves the vehicle driving forward with no key to let go of
        // and no visible cause — the keyup never arrives.
        window.addEventListener("blur", this._releaseAll);
        document.addEventListener("visibilitychange", this._onVisibility);
        this._wrenchTimer = window.setInterval(this._publishWrench, 1000 / WRENCH_HZ);
    }

    stop() {
        window.removeEventListener("keydown", this._onKeyDown);
        window.removeEventListener("keyup", this._onKeyUp);
        window.removeEventListener("blur", this._releaseAll);
        document.removeEventListener("visibilitychange", this._onVisibility);
        window.clearInterval(this._wrenchTimer);
        window.clearInterval(this._depthTimer);
        this._releaseAll();
    }

    _onVisibility = () => {
        if (document.hidden) {
            this._releaseAll();
        }
    };

    _isTypingTarget(target) {
        if (!target) {
            return false;
        }
        const tag = target.tagName;
        return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT"
            || target.isContentEditable;
    }

    _onKeyDown = (event) => {
        // Never steal keys from a number field — the operator is typing a gain,
        // not asking the vehicle to strafe.
        if (this._isTypingTarget(event.target) || event.repeat) {
            return;
        }
        if (event.ctrlKey || event.metaKey || event.altKey) {
            return;
        }
        if (AXES[event.code]) {
            if (!this.getEnabled()) {
                return;
            }
            event.preventDefault();
            this.held.add(event.code);
            this._emitState();
            return;
        }
        if (DEPTH_KEYS[event.code]) {
            if (!this.getEnabled()) {
                return;
            }
            event.preventDefault();
            this.held.add(event.code);
            this._stepDepth(DEPTH_KEYS[event.code]);
            this._startDepthRepeat();
            this._emitState();
        }
    };

    _onKeyUp = (event) => {
        if (!this.held.delete(event.code)) {
            return;
        }
        if (DEPTH_KEYS[event.code] && !this._anyDepthKeyHeld()) {
            window.clearInterval(this._depthTimer);
            this._depthTimer = null;
        }
        this._emitState();
    };

    _releaseAll = () => {
        if (this.held.size === 0) {
            return;
        }
        this.held.clear();
        window.clearInterval(this._depthTimer);
        this._depthTimer = null;
        this._publishWrench();
        this._emitState();
    };

    _anyDepthKeyHeld() {
        return Object.keys(DEPTH_KEYS).some((code) => this.held.has(code));
    }

    _startDepthRepeat() {
        if (this._depthTimer !== null) {
            return;
        }
        this._depthTimer = window.setInterval(() => {
            for (const [code, sign] of Object.entries(DEPTH_KEYS)) {
                if (this.held.has(code)) {
                    this._stepDepth(sign);
                }
            }
        }, DEPTH_REPEAT_MS);
    }

    _stepDepth(sign) {
        const current = Number(this.getTargetDepth());
        const base = Number.isFinite(current) ? current : 0;
        const next = Math.max(0, base + sign * this.depthStepM);
        this.socket.sendTopic(this.protocol.topics.targetDepthM, {
            data: next.toFixed(3),
        });
    }

    /** Compose every held key into one wrench. */
    axes() {
        const totals = {surge: 0, sway: 0, yaw: 0};
        for (const code of this.held) {
            const mapping = AXES[code];
            if (mapping) {
                totals[mapping.axis] += mapping.sign;
            }
        }
        // Opposite keys held together cancel, which is what the operator sees
        // on screen and expects the vehicle to do.
        return {
            surge: Math.sign(totals.surge),
            sway: Math.sign(totals.sway),
            yaw: Math.sign(totals.yaw),
        };
    }

    _publishWrench = () => {
        const enabled = this.getEnabled();
        const axes = enabled ? this.axes() : {surge: 0, sway: 0, yaw: 0};
        const active = axes.surge !== 0 || axes.sway !== 0 || axes.yaw !== 0;

        // Publish while any key is down, plus exactly one zero frame on release
        // so the bus is explicitly cleared instead of relying on the source
        // timeout to fade it out.
        if (!active && !this._wasPublishing) {
            return;
        }
        this._wasPublishing = active;

        this.socket.sendTopic(this.protocol.topics.wrenchCommand, {
            force: {
                x: axes.surge * this.translationForceN,
                y: axes.sway * this.translationForceN,
                z: 0,
            },
            torque: {
                x: 0,
                y: 0,
                z: axes.yaw * this.yawTorqueNm,
            },
        });
    };

    _emitState() {
        this.onState({
            held: new Set(this.held),
            axes: this.axes(),
            enabled: this.getEnabled(),
        });
    }
}

if (typeof window !== "undefined") {
    window.KeyboardPilot = KeyboardPilot;
    window.KeyboardPilotKeys = {AXES, DEPTH_KEYS};
}
