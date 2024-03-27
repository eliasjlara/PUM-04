import numpy as np
from scipy.io.wavfile import write
import rclpy
from rclpy.node import Node
from audio_data.msg import AudioData

class AudioReceiverNode(Node):
    """
    This class represents an audio receiver node that subscribes to audio data and saves it to WAV files.

    Attributes:
        subscription: The subscription object for receiving audio data.
        file_counter: An integer representing the current file counter for naming the output WAV files.

    Methods:
        __init__: Initializes the AudioReceiverNode object.
        listener_callback: The callback function for processing the received audio data.
    """



    def __init__(self) -> None:
        """
        Initializes the AudioReceiverNode object.

        Args:
            None

        Returns:
            None
        """
        super().__init__('audio_receiver_node', namespace='mic')
        self.subscription = self.create_subscription(
            AudioData,
            'mic_audio',
            self.listener_callback,
            10)
        self.subscription
        self.file_counter: int = 0

    def listener_callback(self, msg: AudioData) -> None:
        """
        The callback function for processing the received audio data.

        Args:
            msg: An AudioData object representing the received audio data.

        Returns:
            None
        """
        print("receiving audio data")
        audio_data = np.frombuffer(msg.data, dtype=np.uint8)
        print("reshaping audio data")
        audio_data = audio_data.reshape((msg.samples, msg.channels))
        print("writing to wav-file")
        write(f'output_{self.file_counter}.wav', msg.sample_rate, audio_data)
        self.file_counter += 1

def main(args=None) -> None:
    rclpy.init(args=args)

    audio_receiver_node = AudioReceiverNode()

    try:
        rclpy.spin(audio_receiver_node)
    except KeyboardInterrupt:
        audio_receiver_node.get_logger().info('Audio Receiver: Keyboard interrupt')

    audio_receiver_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
