import atexit

import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayDimension


class AudioListenerNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameters(
            namespace="",
            parameters=[
                ("channels", 1),
                ("frames_per_buffer", 1000),
                ("rate", 16000),
            ],
        )

        self.channels_ = (
            self.get_parameter("channels").get_parameter_value().integer_value
        )
        self.frames_per_buffer_ = (
            self.get_parameter("frames_per_buffer").get_parameter_value().integer_value
        )
        self.rate_ = self.get_parameter("rate").get_parameter_value().integer_value

        self.pyaudio_ = pyaudio.PyAudio()
        self.stream_ = self.pyaudio_.open(
            channels=self.channels_,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.frames_per_buffer_,
            rate=self.rate_,
        )

        self.audio_publisher_ = self.create_publisher(
            Int16MultiArray, "~/audio", 5
        )

        self.audio_publisher_timer_ = self.create_timer(
            float(self.frames_per_buffer_) / float(self.rate_),
            self.audio_publisher_timer_callback_,
        )

        atexit.register(self.cleanup_)

    def audio_publisher_timer_callback_(self) -> None:
        audio = self.stream_.read(self.frames_per_buffer_)
        audio = np.frombuffer(audio, dtype=np.int16)
        audio_msg = Int16MultiArray()
        audio_msg.data = audio.tolist()
        audio_msg.layout.data_offset = 0
        audio_msg.layout.dim.append(
            MultiArrayDimension(label="audio", size=self.frames_per_buffer_, stride=1)
        )
        self.audio_publisher_.publish(audio_msg)

    def cleanup_(self):
        self.stream_.close()
        self.pyaudio_.terminate()


def main(args=None):
    rclpy.init(args=args)
    audio_listener = AudioListenerNode("audio_listener")
    rclpy.spin(audio_listener)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
