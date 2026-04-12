"""
ASR node for Marvin speech project.
Subscribes to Audio messages and processes them for automatic speech recognition.
"""

import re

import rclpy
from rclpy.node import Node

import numpy as np

from audio_msg.msg import Audio
from std_msgs.msg import String

from transformers import AutoProcessor, AutoModelForMultimodalLM

class ASRGemmaNode(Node):
    """ROS2 node for automatic speech recognition."""

    def __init__(self):
        super().__init__('asr_gemma_node')
        
        # Default audio parameters (will be updated from stream)
        self.sample_rate = 16000
        self.channels = 1
        self.bits_per_sample = 16
        self.chunk_size = 512
        self.format: str = ""
        
        # Declare parameters
        self.declare_parameter('device_index', -1)  # -1 for default device
        self.declare_parameter('topic', 'audio_stream')
        self.declare_parameter('output_topic', 'text_stream')
        self.declare_parameter('model_name', 'google/gemma-4-E2B-it')
        
        # Get parameters
        self.device_index = self.get_parameter('device_index').value
        self.topic = self.get_parameter('topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.model_name = self.get_parameter('model_name').value
        self.config_received = False
        
        # Load the ASRmodel
        self.model = AutoModelForMultimodalLM.from_pretrained(self.model_name, dtype="auto", device_map="auto")
        self.processor = AutoProcessor.from_pretrained(self.model_name)
        self.get_logger().info(f'ASR model "{self.model_name}" loaded successfully')

        # Subscribers
        self.chunk_subscriber = self.create_subscription(
            Audio,
            self.topic,
            self.audio_chunk_callback,
            10
        )
        
        # Publishers
        self.text_publisher = self.create_publisher(
            String, 
            self.output_topic, 
            10
        )
        
        # Statistics
        self.chunks_received = 0
        self.chunks_played = 0
        self.buffer_underruns = 0
        
        self.get_logger().info(f'ASR node initialized for topic {self.topic}')

    def audio_chunk_callback(self, msg):
        """Handle incoming audio chunk messages."""
        self.chunks_received += 1

        if msg.info.format != self.format:
            self.get_logger().info(
                f'Audio format set to: {msg.info.format}, '
                f'{msg.info.sample_rate}Hz, {msg.info.num_channels}ch, '
                f'{msg.info.chunk_size} samples/chunk'
            )
            self.format = msg.info.format
            self.sample_rate = msg.info.sample_rate
            self.channels = msg.info.num_channels
            self.chunk_size = msg.info.chunk_size

        # Convert message data to numpy array 
        audio_data = np.array(msg.data.int16_data, dtype=np.int16).astype(np.float32) / 32768.0

        if msg.event == 'start_utterance':
            self.get_logger().info('Start of utterance detected')
            self.reset_buffer()
            self.text_publisher.publish(String(data='<break>'))
            self.append_to_buffer(audio_data)
        elif msg.event == 'end_utterance':
            self.get_logger().info('End of utterance detected')
            self.append_to_buffer(audio_data)
            self.transcribe()
        else:
            if not self.append_to_buffer(audio_data):
                self.get_logger().warning('Audio buffer overflow, processing current buffer')
                # Buffer overflow, should arrange an overlap window but not expecting this to trigger
                self.transcribe()

    def reset_buffer(self):
        self.buffer= np.zeros(30 * self.sample_rate, dtype=np.float32)
        self.buffer_index = 0

    def append_to_buffer(self, audio_data):
        if self.buffer_index + len(audio_data) > len(self.buffer):
            return False
        self.buffer[self.buffer_index:self.buffer_index+len(audio_data)] = audio_data
        self.buffer_index += len(audio_data)
        return True

    def transcribe(self):
        if self.buffer_index > 0:
            buffer = self.buffer
            self.reset_buffer()

            messages = [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "Transcribe the following speech segment in English into English text. Follow these specific instructions for formatting the answer:\n* Only output the transcription, with no newlines.\n* When transcribing numbers, write the digits, i.e. write 1.7 and not one point seven, and write 3 instead of three."},
                        {"type": "audio", "audio": buffer},
                    ]
                }
            ]

            input_ids = self.processor.apply_chat_template(
                    messages,
                    add_generation_prompt=True,
                    tokenize=True, return_dict=True,
                    return_tensors="pt",
            )
            input_ids = input_ids.to(self.model.device, dtype=self.model.dtype)

            outputs = self.model.generate(**input_ids, max_new_tokens=512)

            text = self.processor.batch_decode(
                outputs,
                skip_special_tokens=False,
                clean_up_tokenization_spaces=False
            )
            transcription = _extract_response(text[0])
            self.get_logger().info(f'Transcribed segment: {transcription}')
            self.text_publisher.publish(String(data=transcription))

    def cleanup(self):
        pass

    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()

def _extract_response(text):
    start_delimiter = "<|turn>model"
    end_delimiter = "<turn|>"
    start_index = text.find(start_delimiter)
    if start_index == -1:
        return ""
    text_start = start_index + len(start_delimiter)
    end_index = text.find(end_delimiter, text_start)
    if end_index == -1:
        return ""
    return text[text_start:end_index].strip()

def main(args=None):
    """Main function to run the ASR node."""
    rclpy.init(args=args)
    
    node = ASRGemmaNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ASR node...')
    finally:
        node.cleanup()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            pass

if __name__ == '__main__':
    main()