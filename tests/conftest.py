import asyncio
import logging
import os
import sys
import threading
import time

import pytest
import socketio
import uvicorn
from starlette.applications import Starlette
from starlette.routing import Mount

logging.basicConfig(level=logging.WARNING)

HUB_PORT = 5001
HUB_URL = f"http://localhost:{HUB_PORT}"


@pytest.fixture(scope="session", autouse=True)
def hub_server():
    sys_path = os.path.join(os.path.dirname(__file__), "..", "src")
    if sys_path not in sys.path:
        sys.path.insert(0, sys_path)

    from hub.server import sio_app

    app = Starlette(routes=[Mount("/", app=sio_app)])
    config = uvicorn.Config(app, host="127.0.0.1", port=HUB_PORT, log_level="warning")
    server = uvicorn.Server(config)

    loop = asyncio.new_event_loop()

    def run_server():
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(server.serve())
        except Exception as e:
            print(f"Hub server error: {e}")

    thread = threading.Thread(target=run_server, daemon=True)
    thread.start()

    for _ in range(50):
        try:
            import urllib.request
            urllib.request.urlopen(f"http://127.0.0.1:{HUB_PORT}", timeout=0.5)
            break
        except urllib.error.HTTPError as e:
            if e.code == 404:
                break
            time.sleep(0.1)
        except Exception:
            time.sleep(0.1)
    else:
        raise RuntimeError("Hub server failed to start")

    yield

    server.should_exit = True
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=5)
    time.sleep(0.5)

@pytest.fixture
def make_client(hub_server):
    """Factory that creates connected socket.io clients with message capture."""

    async def _make_client(room=None):
        client = socketio.AsyncClient(logger=False, engineio_logger=False)
        messages = []
        msg_event = asyncio.Event()

        @client.on("message")
        async def on_message(data, binary=None):
            if data.get("room") == room or room is None:
                messages.append(data.get("message", {}))
                msg_event.set()

        await client.connect(HUB_URL)
        await asyncio.sleep(0.1)

        if room:
            await client.emit("join", {"room": room})
            await asyncio.sleep(0.1)

        return {
            "client": client,
            "messages": messages,
            "msg_event": msg_event,
        }

    yield _make_client
