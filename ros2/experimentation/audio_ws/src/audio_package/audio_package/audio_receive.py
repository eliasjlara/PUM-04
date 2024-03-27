import numpy as np
from scipy.io.wavfile import write
import rclpy
from rclpy.node import Node
from audio_data.msg import AudioData
from faster_whisper import WhisperModel

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
        
        default_model = "base.en"
        self.model = WhisperModel(default_model)



    def translate(self, audio_data) -> list:
        """
        Translates the received audio data to text.

        Args:
            audio_data: A 1D numpy array representing the received audio data.

        Returns:
            A list of semgents representing the translated text.
        """

        segments, _ = self.model.transcribe(audio_data, beam_size=5)
        return list(segments)
    

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
            print("the segment contains: ")
            print(segment.text + "\n")
        print("The current message is finished")

    def _msg_to_nparray(self, msg) -> np.ndarray:
        """
        Converts an ros message from topic to an np array for use
        to apply STT

        Args:
            msg: An AudioData object representing the received audio data
        Returns: 
            normalized float32 numpy array
        """
        
        audio_data = np.frombuffer(msg.data, dtype=np.float32)
        # we may need to reshape the data if we have several channels, 
        # not handled yet
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