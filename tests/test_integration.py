import asyncio
import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


@pytest.mark.asyncio
async def test_text_send_receive(make_client):
    sender_ctx = await make_client(room="test_text")
    listener_ctx = await make_client(room="test_text")

    sender = sender_ctx["client"]
    listener = listener_ctx["client"]
    listener_event = listener_ctx["msg_event"]
    listener_messages = listener_ctx["messages"]

    await sender.emit("publish", {
        "room": "test_text",
        "message": {"data": "hello world"},
    })

    await asyncio.wait_for(listener_event.wait(), timeout=5.0)

    assert len(listener_messages) >= 1
    assert listener_messages[0].get("data") == "hello world"

    await sender.disconnect()
    await listener.disconnect()
