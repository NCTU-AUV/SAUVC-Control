// Shared websocket layer for every GUI page.
//
// Replaces the two hand-rolled copies (dashboard and controller each opened
// their own socket, neither reconnected, and the dashboard never showed the
// operator that the link was down). Three things live here because getting any
// of them wrong is invisible until it matters:
//
//   1. Reconnect. A dropped link used to leave the page looking alive with
//      values frozen at whatever arrived last.
//   2. Staleness. Every topic records when it last arrived, so a readout that
//      has stopped updating can be greyed out rather than lying.
//   3. Dispatch. Handlers register by topic name instead of the page walking a
//      chain of a dozen `if` comparisons per message.

const RECONNECT_MIN_MS = 500;
const RECONNECT_MAX_MS = 5000;

class GuiSocket {
    constructor(protocol) {
        this.protocol = protocol;
        this.socket = null;
        this.connected = false;
        this.reconnectDelayMs = RECONNECT_MIN_MS;
        this.reconnectTimer = null;

        this._topicHandlers = new Map();
        this._connectionHandlers = new Set();
        // topic name -> epoch ms of the last message
        this._lastSeen = new Map();
    }

    connect() {
        const url = this.protocol.makeWebsocketUrl(window.location.hostname);
        this.socket = new WebSocket(url, this.protocol.websocketSubprotocol);

        this.socket.onopen = () => {
            this.connected = true;
            this.reconnectDelayMs = RECONNECT_MIN_MS;
            this._emitConnection();
        };

        this.socket.onclose = () => {
            this.connected = false;
            this._emitConnection();
            this._scheduleReconnect();
        };

        // onerror is followed by onclose, so reconnection is handled there.
        this.socket.onerror = () => this.socket.close();

        this.socket.onmessage = (event) => this._handleMessage(event);
    }

    _scheduleReconnect() {
        if (this.reconnectTimer !== null) {
            return;
        }
        const delay = this.reconnectDelayMs;
        // Back off so a server that is down for a while does not get hammered,
        // but stay responsive to the common case of a quick node restart.
        this.reconnectDelayMs = Math.min(delay * 2, RECONNECT_MAX_MS);
        this.reconnectTimer = window.setTimeout(() => {
            this.reconnectTimer = null;
            this.connect();
        }, delay);
    }

    _handleMessage(event) {
        let message;
        try {
            message = JSON.parse(event.data);
        } catch (error) {
            console.warn("gui: unparsable websocket frame", error);
            return;
        }
        if (message.type !== this.protocol.types.topic) {
            return;
        }
        const topicName = message.data?.topic_name;
        if (!topicName) {
            return;
        }
        this._lastSeen.set(topicName, Date.now());
        const handlers = this._topicHandlers.get(topicName);
        if (!handlers) {
            return;
        }
        for (const handler of handlers) {
            try {
                handler(message.data.msg, topicName);
            } catch (error) {
                // One bad handler must not stop the others, or a formatting
                // slip in a minor readout takes the whole page down.
                console.error(`gui: handler for ${topicName} threw`, error);
            }
        }
    }

    /** Register a handler for one topic. Returns an unsubscribe function. */
    onTopic(topicName, handler) {
        if (!this._topicHandlers.has(topicName)) {
            this._topicHandlers.set(topicName, new Set());
        }
        this._topicHandlers.get(topicName).add(handler);
        return () => this._topicHandlers.get(topicName)?.delete(handler);
    }

    /** Register a handler for connect/disconnect. Fires immediately. */
    onConnectionChange(handler) {
        this._connectionHandlers.add(handler);
        handler(this.connected);
        return () => this._connectionHandlers.delete(handler);
    }

    _emitConnection() {
        for (const handler of this._connectionHandlers) {
            handler(this.connected);
        }
    }

    /** Milliseconds since the last message on a topic, or null if never seen. */
    ageMs(topicName) {
        const seen = this._lastSeen.get(topicName);
        return seen === undefined ? null : Date.now() - seen;
    }

    /**
     * True when a topic has gone quiet (or never arrived). Never-seen counts as
     * stale on purpose: "no data yet" and "data stopped" look identical to the
     * operator and both mean the readout cannot be trusted.
     */
    isStale(topicName, thresholdMs) {
        const age = this.ageMs(topicName);
        return age === null || age > thresholdMs;
    }

    send(payload) {
        if (!this.connected || !this.socket) {
            return false;
        }
        this.socket.send(JSON.stringify(payload));
        return true;
    }

    sendTopic(topicName, msg) {
        return this.send(this.protocol.makeTopicMessage(topicName, msg));
    }

    sendAction(actionName, data = {}) {
        return this.send(this.protocol.makeActionMessage(actionName, data));
    }

    sendController(group, action, data = {}) {
        return this.send(this.protocol.makeControllerMessage(group, action, data));
    }
}

if (typeof window !== "undefined") {
    window.GuiSocket = GuiSocket;
}
