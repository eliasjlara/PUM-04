import numpy as np
import rclpy
from rclpy.node import Node
from audio_data.msg import AudioData
from aida_interfaces.srv import SetState
from std_msgs.msg import String
from faster_whisper import WhisperModel



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
        init_services : Initializes the services for the node for turning on and of the transcription
        set_state_of_node : Sets the desired state for the node when request is recieved
        
        translate: Translates audio from numpy data to text
        _message_to_numpy_array : Translates the message from topic to numpy array
        destroy_node : Destructs the STTNode object.
        publish_result : Publishes the inputed string to stt_result topic
    """

    def __init__(self, model_size: str = "tiny.en"):
        """
        Initializes the STTNode object.

        Args:
            None

        Returns:
            None
        """

        # We may not need the namespace variable here
        # Important! For all subscribers and publishers the namespace
        # need to be the same to work properly
        super().__init__('audio_receiver_node')

        # Init publisher of finished result
        # Publish the data to the topic called STT_result
        self.publisher = self.create_publisher(String, 'stt/stt_result', 10)


        # Init subscriber to mic data
        # Subscribes to the topic called mic_audio
        self.subscription = self.create_subscription(
            AudioData,
            'mic/mic_audio',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        
        # Init the STT model
        self.stt_model = WhisperModel(model_size)

        # Init the services for the node
        self.init_services()

        # Set the node status to active by default
        # Trancsription is done when the node is active
        self.active = True


    def listener_callback(self, msg):
        """
        The callback function for processing the received audio data.

        Args:
            msg: An AudioData object representing the received audio data.

        Returns:
            None
        """

        if self.active:
            self.get_logger().info("STT node: Receiving audio data from topic")
            audio_data = self._message_to_numpy_array(msg)
            self.get_logger().info("STT node: Applying STT model to audio data")
            #translation = self.stt_model.transcribe_audio(audio_data)
            translation = self.translate(audio_data)
            #for segment in translation:
                # The segment is the result of the transcription, strip of trailing and leading whitespaces
            #    result = segment.text.strip() 
            #    self.get_logger().info("STT node: Transcription contains segment: " + result)
            #    self.publish_result(result)
            self.publish_result(translation)
            self.get_logger().info("STT node: The current transcription is finished")
        else:
            self.get_logger().info("STT node: Node is idle, no transcription is done")

    def _message_to_numpy_array(self, msg) -> np.ndarray:
        """ 
        Converts an ros message from topic to an np array for use
        to apply STT
        
        Args:
            msg: An AudioData object representing the received audio data
        
        Returns: 
            Float32 numpy array in the range [-1, 1]
        """
        if msg.samples == 0:
            raise ValueError("No audio data in message")
        #TODO : Add error handling
        return np.frombuffer(msg.data, dtype=np.float32)
    

    def translate(self, audio_data: np.ndarray) -> str:
        """
        Translates audio from numpy data to text

        Args:
            audio_data: A numpy array containing the audio data to be translated
        
        Returns:
            A string containing the translated text
        """
        segments, _ = self.stt_model.transcribe(audio_data, beam_size=5)
        list_of_segments = list(segments)
        return list_of_segments[0].text.strip()

    def destroy_node(self):
        """
        Destructs the STTNode object.

        Args:
            None

        Returns:
            None
        """
    
        self.subscription.destroy()
        self.publisher.destroy()
        self.srv.destroy()
        self.stt_model = None
        super().destroy_node()
    
    def init_services(self) -> None:
        """
        Initialzes the services for the node.
        Uses the SetState service to start and stop the recording.

        Args:
            None
        
        Returns:
            None
        """
        self.srv = self.create_service(SetState, 'stt/SetState', self.set_state_of_node)
    
    def set_state_of_node(self, request: SetState.Request, response: SetState.Response) -> SetState.Response:
        """
        Sets the desired state for node when request is recieved.

        #TODO finish this comment
        """

        try:
            if request.desired_state == "active":
                self.active = True
                self.get_logger().info("STT node: Transcription node is active.")
            elif request.desired_state == "idle":
                self.active = False
                self.get_logger().info("STT node: Transcription node is idle.")
            response.message = f"Successfully set state to: {request.desired_state}"
            response.success = True
        except Exception as e:
            response.message = f"Error setting state: {request.desired_state} - {e}"
            response.success = False
        return response
    

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