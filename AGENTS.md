# AGENTS.md - Marvin Base Station (socket.io)

## Project Overview
Base station nodes for Marvin the droid. Nodes communicate via python-socket.io through a central hub process.

## Architecture
- **Hub**: Standalone ASGI server (uvicorn + python-socketio) on port 5000. Manages room subscriptions and routes messages.
- **Rooms**: Equivalent of ROS2 topics. Join to subscribe, emit to publish.
- **Messages**: Pydantic models matching existing ROS2 msg fields. Sent as dicts over socket.io. Audio data uses binary attachments.
- **Nodes**: Separate asyncio processes. Each connects to hub, subscribes to input rooms, publishes to output rooms. Config via CLI args with env var fallback.

## Socket.IO Protocol
| Event | Direction | Payload |
|-------|-----------|---------|
| `join` | client→hub | `{"room": "text_stream"}` |
| `leave` | client→hub | `{"room": "text_stream"}` |
| `publish` | client→hub | `{"room": "...", "message": {...}, "has_binary": true?}` + optional binary |
| `message` | hub→clients | `{"room": "...", "message": {...}}` (+ binary if present), sender excluded |

## Setup
```bash
python3 -m venv ./venv
. ./venv/bin/activate
pip install -r requirements.txt
. ./setup.sh
```

## Running
```bash
# Start hub first
./start_hub.sh

# Then individual nodes (all source setup.sh)
./start_asr.sh         # ASR node (whisper)
./start_asr_gemma.sh   # Alternative ASR using gemma
./start_llm.sh         # LLM node (gemma4:26b via ollama)
./start_tts.sh         # TTS node (pocket-tts)
./start_player.sh      # Audio player node
./start_capture.sh     # Audio capture node
```

## Package Structure (`src/`)
| Package | Purpose |
|---------|---------|
| `hub/` | Hub ASGI server (`server.py`) |
| `messages/` | Shared pydantic models + BaseNode class |
| `audio_base/` | capture_node, player_node, asr_node, asr_gemma_node, tts_node |
| `llm_support/` | llm_node |
| `tools/` | CLI utilities (listener, sender) |

## Key Rooms & Data Flow
- `audio_stream` → ASR node → `text_stream` → LLM node → `llm_response` → TTS node → `speech_stream` → player
- `events` room for debugging and stop/interrupt signals (nodes opt-in)
- ASR node publishes `stop` event when it transcribes the word "stop"

## BaseNode Pattern
```python
class MyNode(BaseNode):
    def __init__(self, ...):
        super().__init__(hub_url=..., node_name="my_node")
        self.handler("some_room")(self.on_some_message)

    async def on_some_message(self, data: dict, binary: bytes | None):
        ...

    async def run(self):
        await self.sio.connect(self.hub_url)
        await self._connected.wait()
        await self.subscribe("some_room")
        # ... keep alive loop
```

## Important Quirks
- **Hub URL**: Defaults to `http://localhost:5000`, override with `--hub-url` or `HUB_URL` env var
- **pywhispercpp GPU**: CUDA setup is unreliable; Strix Halo uses `GGML_VULKAN=1`
- **Audio payloads**: Binary attachments for int16 data (not JSON arrays)
- **Events room**: Treated like any other room - nodes explicitly subscribe if needed
- **TTS synthesis**: Runs in a background thread to avoid blocking the asyncio event loop
- **ASR transcription**: Also runs in a thread executor for the same reason
- **LLM streaming**: Uses `asyncio.create_task()` to avoid blocking message handlers

## Testing
No unit tests yet. Manual testing:
```bash
# Start hub, then in separate terminals:
./start_hub.sh
python src/tools/listener.py --room test_topic
python src/tools/sender.py --room test_topic --message "hello"
```

## Gitignore
Ignores: `venv`, `build`, `install`, `log`, `*.pyc`, `__pycache__`
