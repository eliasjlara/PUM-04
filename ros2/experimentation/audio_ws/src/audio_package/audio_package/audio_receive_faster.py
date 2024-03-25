import numpy as np
from faster_whisper import WhisperModel
#import decodeAudio
import rclpy
from rclpy.node import Node
from audio_data.msg import AudioData




class AudioReceiverSTT(Node):
    """
    This class represents an audio receiver that subscribes to audio data and translates it to text.

    Attributes:
        model: A WhisperModel object representing the model for translating audio data to text.

    Methods:
        __init__: Initializes the AudioReceiverSTT object.
        translate: Translates the received audio data to text.
    """

    def __init__(self, model_size: str=None):
        """
        Initializes the AudioReceiverSTT object.

        Args:
            model_size: A string representing model to be used for translating audio data to text.

        Returns:
            None
        """
        if model_size is not None:
            self.model = WhisperModel(model_size, device="auto")
        else:
            default_model = "base.en"
            self.model = WhisperModel(default_model)


        super().__init__('audio_receiver_node', namespace='mic')
        self.subscription = self.create_subscription(
            AudioData,
            'mic_audio',
            self.listener_callback,
            10)
        self.subscription
        self.file_counter = 0

    def translate(self, audio_data) -> list:
        """
        Translates the received audio data to text.

        Args:
            audio_data: A 2D numpy array representing the received audio data.

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
        print("message received: \n")
        audio_data = self._msg_to_nparray(msg)
        translation = self.translate(audio_data)
        for segment in translation:
            print(segment.text + "\n")
    
    
    def _msg_to_nparray(self, msg) -> np.ndarray:
        """
        Converts an ros message from topic to an np array for use
        to apply STT

        Returns: 
            normalized float32 numpy array
        """
        
        audio_data = np.frombuffer(msg.data, dtype=np.int32)

        audio_data = audio_data.astype(np.float32)
        audio_data = audio_data / 2**16
        return audio_data


def main(args=None):
    rclpy.init(args=args)

    audio_receiver_node = AudioReceiverSTT()

    try:
        rclpy.spin(audio_receiver_node)
    except KeyboardInterrupt:
        audio_receiver_node.get_logger().info('Audio Receiver: Keyboard interrupt')

    audio_receiver_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()