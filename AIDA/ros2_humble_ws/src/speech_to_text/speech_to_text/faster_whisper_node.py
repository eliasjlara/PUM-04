import numpy as np
import rclpy
from rclpy.node import Node
from audio_data.msg import AudioData
from faster_whisper import WhisperModel
from std_msgs.msg import String


class STTNode(Node):
    """
    This class represents an node that subscribes to audio data topic and converts 
    the audio to text. Thereafter it published the finished result as string to 
    stt_result topic.

    Attributes:
        subscription: The subscription object for receiving audio data.
        publisher : The publisher object for sending the finished STT result
    Methods:
        __init__: Initializes the STTNode object.
        listener_callback: The callback function for processing the received audio data.
        translate: Translates audio from numpy data to text
        _msg_to_nparray : Translates the message from topic to numpy array
        publish_result : Publishes the inputed string to stt_result topic
    """

    def __init__(self):
        """
        Initializes the STTNode object.

        Args:
            None

        Returns:
            None
        """

        # We may not need the namespace variable here
        # Important! For all subscribers and publishers the namespace
        # need to be the same to work
        super().__init__('audio_receiver_node', namespace='mic')

        # Init publisher of finished result

        # Publish the data to the topic called STT_result
        self.publisher = self.create_publisher(String, 'stt_result', 10)


        # Init subscriber to mic data
        # Subscribes to the topic called mic_audio
        
        self.subscription = self.create_subscription(
            AudioData,
            'mic_audio',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        
        #The current model for use with faster_whisper
        default_model = "base.en"
        self.model = WhisperModel(default_model)

    def translate(self, audio_data) -> list:
        """
        Translates the received audio data to text.

        Args:
            audio_data: A 1D numpy array representing the received audio data.
        Returns:
            A list of segments representing the translated text.
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

        self.get_logger().info("STT node: Receiving audio data from topic")
        audio_data = self._msg_to_nparray(msg)
        self.get_logger().info("STT node: Applying STT model to audio data")
        translation = self.translate(audio_data)
        for segment in translation:
            result = segment.text
            self.get_logger().info("STT node: Transcription contains segment: " + result)
            self.publish_result(result)
        self.get_logger().info("STT node: The current transcription is finished")

    def _msg_to_nparray(self, msg) -> np.ndarray:
        """
        Converts an ros message from topic to an np array for use
        to apply STT

        Args:
            msg: An AudioData object representing the received audio data
        Returns: 
            Float32 numpy array in the range [-1, 1]
        """
        
        audio_data = np.frombuffer(msg.data, dtype=np.float32)
        # we may need to reshape the data if we have several channels, 
        # not handled yet
        return audio_data
    
    def publish_result(self, result : str):
        """'
        Publishes the speech to text result to topic

        Args: 
            result: A string containing the STT result to be published to topic

        Returns:
            None 
        """
        msg = String()
        msg.data = result
        self.get_logger().info('STT node: Publishing result to stt_result topic')
        self.publisher.publish(msg)
        self.get_logger().info('STT node: Finished publishing result to stt_result topic')    

def main(args=None):
    rclpy.init(args=args)

    stt_node = STTNode()

    try:
        rclpy.spin(stt_node)
    except KeyboardInterrupt:
        stt_node.get_logger().info('STT node: Keyboard interrupt')

    stt_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()