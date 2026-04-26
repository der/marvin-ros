# Marvin Base Station

Socket.io-based base station nodes for Marvin the droid. All nodes communicate via a central hub process using python-socketio.

## Architecture

- **Hub**: Standalone ASGI server (uvicorn + python-socketio) on port 5000
- **Rooms**: Equivalent of ROS2 topics - join to subscribe, emit to publish
- **Messages**: Pydantic models sent as dicts over socket.io, with binary attachments for audio data
- **Nodes**: Separate asyncio processes connecting to the hub

## Setup

```bash
python3 -m venv ./venv && touch ./venv/COLCON_IGNORE
. ./venv/bin/activate
pip install -r requirements.txt
. ./setup.sh
```

For ASR node, install `pywhispercpp` with GPU support:

```bash
# Strix Halo (Vulkan)
GGML_VULKAN=1 pip install git+https://github.com/absadiki/pywhispercpp
```

## Running

```bash
# Start hub first
./start_hub.sh

# Then individual nodes
./start_capture.sh     # Audio capture from mic
./start_asr.sh         # ASR (whisper)
./start_asr_gemma.sh   # ASR (gemma multimodal)
./start_llm.sh         # LLM (gemma4:26b via ollama)
./start_tts.sh         # TTS (pocket-tts)
./start_player.sh      # Audio playback
```

## Rooms & Data Flow

```
audio_stream → ASR node → text_stream → LLM node → llm_response → TTS node → speech_stream → player
```

| Room | Direction | Content |
|------|-----------|---------|
| `audio_stream` | capture → ASR | Audio chunks (AudioMessage) |
| `text_stream` | ASR → LLM | Transcribed text |
| `llm_response` | LLM → TTS | LLM response sentences |
| `speech_stream` | TTS → player | Synthesized audio chunks |
| `events` | any → any | Debug/stop/interrupt signals |

## Nodes

### capture_node
Captures audio from default mic (16kHz, mono, 16-bit, 512-sample chunks) and publishes to `audio_stream`. Utterance start/end events come from an external source.

### player_node
Subscribes to an audio room and plays through default speaker using PyAudio. Handles dynamic format changes.

### asr_node
Subscribes to `audio_stream`, buffers audio between start/end utterance events, transcribes using pywhispercpp, publishes to `text_stream` and `events`.

### asr_gemma_node
Alternative ASR using Gemma multimodal model instead of whisper.

### tts_node
Subscribes to `text_stream`, synthesizes speech using pocket-tts, publishes audio chunks to `speech_stream`. Handles stop/interrupt events.

### llm_node
Subscribes to `text_stream`, streams responses from Ollama via pydantic_ai, publishes sentence-by-sentence to `llm_response`. Handles stop/interrupt events.

## CLI Tools

```bash
# Listen on any room
python src/tools/listener.py --room text_stream

# Send text to any room
python src/tools/sender.py --room text_stream --message "hello"
```

## Configuration

All nodes accept `--hub-url` (env: `HUB_URL`, default: `http://localhost:5000`) plus node-specific options. Run with `--help` for details.

## Audio Message Format

| Field | Type | Description |
|-------|------|-------------|
| `info.num_channels` | int | Channel count (default: 1) |
| `info.sample_rate` | int | Sample rate (default: 16000) |
| `info.chunk_size` | int | Samples per chunk (default: 512) |
| `info.format` | str | Format identifier |
| `data.int16_data` | list[int] | Audio samples as int16 |
| `data.float32_data` | list[float] | Audio samples as float32 |
| `event` | str | `start_utterance`, `end_utterance`, `break`, or empty |
