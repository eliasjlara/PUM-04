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

    def __init__(self):
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
        self.file_counter = 0

    def listener_callback(self, msg):
        """
        The callback function for processing the received audio data.

        Args:
            msg: An AudioData object representing the received audio data.

        Returns:
            None
        """
        
        print("receiving audio data")
        audio_data = self._msg_to_nparray(msg)
        translation = self.translate(audio_data)
        for segment in translation:
            print(segment.text + "\n")
        
        
        audio_data = np.frombuffer(msg.data, dtype=np.uint8)
        print("reshaping audio data")
        audio_data = audio_data.reshape((msg.samples, msg.channels))
        print("writing to wav-file")
        write(f'output_{self.file_counter}.wav', msg.sample_rate, audio_data)
        self.file_counter += 1

    def _msg_to_nparray(self, msg) -> np.ndarray:
        """
        Converts an ros message from topic to an np array for use
        to apply STT

        Args:
            msg: An AudioData object representing the received audio data
        Returns: 
            normalized float32 numpy array
        """
        
        audio_data = np.frombuffer(msg.data, dtype=np.int32)
        audio_data = audio_data.astype(np.float32)
        # we may need to reshape the data if we have several channels, 
        # not handled yet
        audio_data = audio_data / 2**16
        return audio_data

def main(args=None):
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