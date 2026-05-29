import aiohttp
from aiohttp import web
import asyncio
import json
import threading
from pathlib import Path
import gui
from . import protocol

PACKAGE_DIR = Path(gui.__file__).parent
STATIC_DIR = PACKAGE_DIR / "static"
DASHBOARD_DIR = STATIC_DIR / "dashboard"
CONTROLLER_DIR = STATIC_DIR / "controller"


class AIOHTTPServer:
    async def respond_index(request):
        return web.FileResponse(path=DASHBOARD_DIR / "index.html")

    async def respond_controller(request):
        return web.FileResponse(path=CONTROLLER_DIR / "controller.html")

    def send_topic(self, topic_name, msg):
        payload = json.dumps(protocol.topic_payload(topic_name, msg))

        # Keep only the latest payload per topic while disconnected.
        with self._pending_lock:
            if (
                not self.event_loop
                or self.websocket_response is None
                or self.websocket_response.closed
            ):
                self._pending_topic_payloads[topic_name] = payload
                return
            loop = self.event_loop
            websocket_response = self.websocket_response

        asyncio.run_coroutine_threadsafe(websocket_response.send_str(payload), loop)

    async def websocket_handler(self, request):
        websocket_response = web.WebSocketResponse(
            protocols=(protocol.WEBSOCKET_SUBPROTOCOL,)
        )
        await websocket_response.prepare(request)

        # Flush the latest pending message for each topic accumulated before connect.
        with self._pending_lock:
            self.websocket_response = websocket_response
            pending_payloads = list(self._pending_topic_payloads.values())
            self._pending_topic_payloads = {}
        for payload in pending_payloads:
            await websocket_response.send_str(payload)

        async for msg in websocket_response:
            if msg.type == aiohttp.WSMsgType.TEXT:
                if msg.data == 'close':
                    await websocket_response.close()
                else:
                    self._msg_callback(msg.data)
            elif msg.type == aiohttp.WSMsgType.ERROR:
                print(
                    "websocket connection closed with exception %s"
                    % websocket_response.exception()
                )

        print('websocket connection closed')

        return websocket_response

    @web.middleware
    async def no_cache_middleware(request, handler):
        response = await handler(request)
        response.headers['Cache-Control'] = 'no-store'
        return response

    def __init__(self, msg_callback):
        self._msg_callback = msg_callback

        app = web.Application(middlewares=[AIOHTTPServer.no_cache_middleware])

        app.router.add_get(protocol.WEBSOCKET_PATH, self.websocket_handler)
        app.router.add_get('/', AIOHTTPServer.respond_index)
        app.router.add_get('/controller', AIOHTTPServer.respond_controller)
        app.router.add_get('/play', AIOHTTPServer.respond_controller)
        app.router.add_static('/static/', STATIC_DIR)

        self.runner = web.AppRunner(app)
        self.websocket_response = None
        self.event_loop = None
        self._pending_topic_payloads = {}
        self._pending_lock = threading.Lock()
        self._thread = None
        self._stopped = False

    async def start(self):
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, '0.0.0.0', 80)

        await self.site.start()

    async def stop(self):
        if self._stopped:
            return
        self._stopped = True
        await self.runner.cleanup()

    def run_loop(self):
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        self.event_loop.run_until_complete(self.start())
        try:
            self.event_loop.run_forever()
        finally:
            # Ensure cleanup if stop_threading did not already run it.
            self.event_loop.run_until_complete(self.stop())
            self.event_loop.close()

    def start_threading(self):
        self._thread = threading.Thread(
            target=self.run_loop,
            daemon=True,
            name="AIOHTTPServerThread",
        )
        self._thread.start()

    def stop_threading(self, join_timeout: float = 2.0):
        loop = self.event_loop
        if not loop:
            return
        asyncio.run_coroutine_threadsafe(self.stop(), loop)
        loop.call_soon_threadsafe(loop.stop)

        if self._thread:
            self._thread.join(timeout=join_timeout)


if __name__ == "__main__":
    aiohttp_server = AIOHTTPServer(lambda msg: print(msg))
    aiohttp_server.start_threading()

    while True:
        pass
