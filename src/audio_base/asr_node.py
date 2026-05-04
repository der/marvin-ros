"""ASR node for Marvin speech project.

Ported from ROS2 to socket.io. Subscribes to audio_stream room and publishes
transcriptions to text_stream room using pywhispercpp.
"""

import argparse
import asyncio
import contextlib
import logging
import os
import threading
from queue import Empty, Queue

import numpy as np
from pywhispercpp.model import Model

from messages.audio import AudioInfo, AudioMessage
from messages.base import BaseNode

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
logger = logging.getLogger("asr_node")


class ASRNode(BaseNode):
    def __init__(
        self,
        hub_url: str,
        topic: str = "/audio_stream",
        output_topic: str = "/text_stream",
        model_name: str = "large-v3-turbo-q5_0",
    ):
        super().__init__(hub_url=hub_url, node_name="asr_node")
        self.topic = topic
        self.output_topic = output_topic
        self.model_name = model_name

        # Audio state
        self.sample_rate = 16000
        self.channels = 1
        self.chunk_size = 512
        self.format: str = ""

        # Model
        logger.info(f'Loading Whisper model "{self.model_name}"...')
        self.model: Model = Model(self.model_name)
        logger.info(f'Whisper model "{self.model_name}" loaded successfully')

        # Buffer for accumulating audio
        self.buffer: np.ndarray= np.zeros(30 * self.sample_rate, dtype=np.float32)
        self.buffer_index = 0
        self.buffer_lock = threading.Lock()

        # Results queue for publishing transcriptions
        self._results_queue: Queue = Queue()

        # Register handler
        self.handler(self.topic)(self.audio_chunk_callback)

    def reset_buffer(self):
        with self.buffer_lock:
            self.buffer = np.zeros(30 * self.sample_rate, dtype=np.float32)
            self.buffer_index = 0

    def append_to_buffer(self, audio_data):
        with self.buffer_lock:
            if self.buffer_index + len(audio_data) > len(self.buffer):
                return False
            self.buffer[self.buffer_index : self.buffer_index + len(audio_data)] = audio_data
            self.buffer_index += len(audio_data)
            return True

    def _transcribe_sync(self):
        """Run transcription in a thread. Puts results into _results_queue."""
        with self.buffer_lock:
            if self.buffer_index == 0:
                return
            buffer = self.buffer[: self.buffer_index].copy()
        self.reset_buffer()

        def segment_callback(segment):
            text = segment.text
            logger.info(f"Transcribed segment: {text}")
            self._results_queue.put_nowait(text)

        self.model.transcribe(buffer, new_segment_callback=segment_callback)

    async def audio_chunk_callback(self, message: dict):
        """Handle incoming audio chunk messages."""
        info = message.get("info", {})
        data = message.get("data", {})
        event = message.get("event", "")

        fmt = info.get("format", "")
        if fmt != self.format:
            logger.info(
                f"Audio format set to: {fmt}, "
                f"{info.get('sample_rate')}Hz, {info.get('num_channels')}ch, "
                f"{info.get('chunk_size')} samples/chunk"
            )
            self.format = fmt
            self.sample_rate = info.get("sample_rate", self.sample_rate)
            self.channels = info.get("num_channels", self.channels)
            self.chunk_size = info.get("chunk_size", self.chunk_size)

        # Convert to numpy array
        int16_list = data.get("int16_data", [])
        if not int16_list:
            return
        audio_data = np.array(int16_list, dtype=np.int16).astype(np.float32) / 32768.0

        if event == "start_utterance":
            logger.info("Start of utterance detected")
            self.reset_buffer()
            self.append_to_buffer(audio_data)
        elif event == "end_utterance":
            logger.info("End of utterance detected")
            self.append_to_buffer(audio_data)
            await asyncio.to_thread(self._transcribe_sync)
        else:
            if not self.append_to_buffer(audio_data):
                logger.warning("Audio buffer overflow, processing current buffer")
                await asyncio.to_thread(self._transcribe_sync)

    async def _publisher_loop(self):
        """Publish transcription results from the thread-safe queue."""
        while True:
            try:
                text = self._results_queue.get_nowait()
                await self.publish(self.output_topic, text)
                await self.publish_event("asr", text)
                if text.strip().lower().replace(".", "") == 'stop':
                    await self.publish_event("sys", "stop")
                self._results_queue.task_done()
            except Empty:
                await asyncio.sleep(0.1)

    async def run(self):
        # Connect to hub
        await self.sio.connect(self.hub_url)
        await self._connected.wait()

        # Subscribe
        await self.subscribe(self.topic)

        # Start publisher loop
        publisher_task = asyncio.create_task(self._publisher_loop())

        logger.info(f"ASR node initialized for topic {self.topic}")

        try:
            while self.sio.connected:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
        finally:
            publisher_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await publisher_task
            await self.sio.disconnect()


def main():
    parser = argparse.ArgumentParser(description="ASR node")
    parser.add_argument("--hub-url", default=None, help="Hub URL")
    parser.add_argument("--topic", default="/audio_stream", help="Audio room to listen to")
    parser.add_argument("--output-topic", default="/text_stream", help="Text room to publish to")
    parser.add_argument("--model-name", default="large-v3-turbo-q5_0", help="Whisper model name")
    args = parser.parse_args()

    hub_url = args.hub_url or os.environ.get("HUB_URL", "http://localhost:5000")
    node = ASRNode(
        hub_url=hub_url,
        topic=args.topic,
        output_topic=args.output_topic,
        model_name=args.model_name,
    )
    asyncio.run(node.run())


if __name__ == "__main__":
    main()
