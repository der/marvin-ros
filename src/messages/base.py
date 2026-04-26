"""Base node class and text message model for socket.io communication."""

import asyncio
import logging
import os
from typing import Any, Callable, Optional

import socketio
from pydantic import BaseModel

logger = logging.getLogger("node")


class TextMessage(BaseModel):
    """Plain text message, equivalent to std_msgs/msg/String."""

    data: str


class BaseNode:
    """Base class for nodes that communicate via socket.io hub.

    Usage:
        class MyNode(BaseNode):
            def __init__(self, ...):
                super().__init__(hub_url=..., node_name="my_node")
                self.handler("some_room")(self.on_some_message)

            async def on_some_message(self, data: dict, binary: bytes | None):
                ...

        node = MyNode()
        await node.subscribe("some_room")
        await node.run()
    """

    def __init__(
        self,
        hub_url: Optional[str] = None,
        node_name: str = "node",
    ):
        self.hub_url = hub_url or os.environ.get("HUB_URL", "http://localhost:5000")
        self.node_name = node_name
        self.sio = socketio.AsyncClient(logger=False, engineio_logger=False)
        self._handlers: dict[str, Callable] = {}
        self._connected = asyncio.Event()

        @self.sio.event
        async def connect():
            logger.info(f"{self.node_name} connected to hub: {self.hub_url}")
            self._connected.set()

        @self.sio.event
        async def connect_error(data):
            logger.error(f"{self.node_name} connection failed: {data}")

        @self.sio.event
        async def disconnect():
            logger.info(f"{self.node_name} disconnected from hub")
            self._connected.clear()

        @self.sio.on("message")
        async def on_message(data: dict):
            room = data.get("room", "unknown")
            message = data.get("message", {})
            binary = data.get("binary")
            handler = self._handlers.get(room)
            if handler:
                await handler(message, binary)
            else:
                logger.debug(f"{self.node_name} received message on {room} (no handler)")

    def handler(self, room: str):
        """Decorator to register a handler for messages on a room.

        Handler signature: async def handler(message: dict, binary: bytes | None)
        """

        def decorator(fn: Callable):
            self._handlers[room] = fn
            return fn

        return decorator

    async def subscribe(self, room: str) -> None:
        """Join a room to receive messages."""
        await self.sio.emit("join", {"room": room})
        logger.info(f"{self.node_name} subscribed to room: {room}")

    async def publish(
        self,
        room: str,
        message: dict,
        binary: Optional[bytes] = None,
    ) -> None:
        """Publish a message to a room."""
        data: dict[str, Any] = {"room": room, "message": message}
        if binary is not None:
            data["has_binary"] = True
            data["binary"] = binary
        await self.sio.emit("publish", data)

    async def run(self) -> None:
        """Connect to hub and wait until interrupted."""
        await self.sio.connect(self.hub_url)
        await self._connected.wait()
        try:
            while self.sio.connected:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
        finally:
            await self.sio.disconnect()

    async def shutdown(self) -> None:
        """Disconnect from hub."""
        if self.sio.connected:
            await self.sio.disconnect()
