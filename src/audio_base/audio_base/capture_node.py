"""
Audio capture node for Marvin speech project.
Captures audio at 16kHz in 512-sample chunks and publishes AudioStamped messages.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import pyaudio
import numpy as np

from audio_msg.msg import AudioStamped, Audio, AudioInfo, AudioData
from std_msgs.msg import Header

class AudioCaptureNode(Node):
    """ROS2 node for audio capture using PyAudio."""

    def __init__(self):
        super().__init__('audio_capture')
        
        # Declare parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_size', 512)
        self.declare_parameter('device_index', -1)  # -1 for default device
        self.declare_parameter('topic', 'audio_stream')
        
        # Get parameters
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value  
        self.chunk_size = self.get_parameter('chunk_size').value
        self.device_index = self.get_parameter('device_index').value
        self.topic = self.get_parameter('topic').value
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Publishers
        self.chunk_publisher = self.create_publisher(
            Audio, 
            self.topic, 
            10
        )
        
        # Audio stream
        self.stream = None
        self.is_streaming = False
        self.init_audio_stream()
        
        # Timer for status logging
        self.create_timer(5.0, self.log_status)
        
        self.get_logger().info(
            f'Audio capture node sending to {self.topic}: '
            f'{self.sample_rate}Hz, {self.channels}ch, '
            f'{self.chunk_size} samples/chunk'
        )

    def init_audio_stream(self):
        """Initialize the audio input stream."""
        try:
            device_index = self.device_index if self.device_index >= 0 else None
            
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size,
                input_device_index=device_index,
                stream_callback=self.audio_callback
            )
            
            self.stream.start_stream()
            self.is_streaming = True
            
            self.get_logger().info('Audio stream started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize audio stream: {e}')
            self.is_streaming = False

    def audio_callback(self, in_data, frame_count, time_info, status):
        """PyAudio callback function for processing audio chunks."""
        if status:
            self.get_logger().warn(f'Audio stream status: {status}')
        
        audio_data = np.frombuffer(in_data, dtype=np.int16)
        
        msg = Audio()
        # msg.header = Header()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = 'audio_capture'

        msg.info = AudioInfo()
        msg.info.sample_rate = self.sample_rate
        msg.info.num_channels = self.channels
        msg.info.chunk_size = len(audio_data)
        msg.data = AudioData()
        msg.data.int16_data = audio_data.tolist()
        
        # Publish the chunk
        self.chunk_publisher.publish(msg)
        
        return (None, pyaudio.paContinue)

    def log_status(self):
        """Log periodic status information."""
        status = "streaming" if self.is_streaming else "stopped"
        self.get_logger().info(f'Audio capture status: {status}')

    def cleanup(self):
        """Clean up resources."""
        if self.stream is not None:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except Exception as e:
                pass
        
        if self.audio is not None:
            self.audio.terminate()
        
        self.get_logger().info('Audio capture cleanup completed')

    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()


def main(args=None):
    """Main function to run the audio capture node."""
    rclpy.init(args=args)
    
    node = AudioCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down audio capture node...')
    finally:
        node.cleanup()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            pass


if __name__ == '__main__':
    main()
