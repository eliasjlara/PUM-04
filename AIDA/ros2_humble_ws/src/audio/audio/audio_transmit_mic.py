import queue
import threading
import sounddevice as sd
import time
import numpy as np

import rclpy
from rclpy.node import Node
from audio_data.msg import AudioData 
from aida_interfaces.srv import SetState


class AudioTransmitterNode(Node):
    """
    A ROS2 node for transmitting audio data.

    This node initializes a publisher to publish audio data and captures audio from the microphone.
    It provides methods to start and stop the capture and publisher workers.

    Attributes: 

    Methods: 
        __init__ : Initializes the ros node, as well as the publisher and capturer of mic data
        destroy_node : Destroys the ros node when finished
        initialize_publisher : Initializes the publisher, called in init
        initialize_capture_queue : Initializes the capture queue for use when saving mic data
        stop_workers : Stops the publisher and capture queue
        record_audio : records the audio from mic
        _to_msg : Converts the nparray to an audio message to be published to topic
        publish_audio : Publishes the audio message to the topic
    """

    def __init__(self):
        """
        Initializes the AudioTransmitterNode.

        This method sets up the ROS2 node, initializes the publisher, and sets up the capture queue.
        """
        super().__init__('ros2_mic_node', namespace='mic')

        self.declare_parameter("audio_topic", "mic_audio")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("duration", 5.0)
        self.declare_parameter("capture_once", True)

        self.init_services()
        self.initialize_publisher()
        self.initialize_capture_queue()

    def destroy_node(self):
        """
        Destroys the AudioTransmitterNode.

        This method cleans up and destroys the ROS2 node.

        Args:
            None

        Returns:
            None
        """
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
        self.srv = self.create_service(SetState, 'SetState', self.set_recording)

    def initialize_publisher(self):
        """
        Initializes the publisher.

        This method creates a publisher object for publishing audio data.

        Args:
            None

        Returns:
            None
        """
        self.publisher = self.create_publisher(AudioData, self.get_parameter('audio_topic').get_parameter_value().string_value, 10)
        self.frame_num = 0

    def initialize_capture_queue(self):
        """
        This method initializes the capture queue and sets the lock for the queue.

        Args:
            None

        Returns:
            None
        """
        self.queue_lock = threading.Lock()

        self.capture_queue = queue.Queue()

    def start_workers(self) -> None:
        """
        Starts the capture and publisher workers.

        This method creates the capture and publisher events and threads.
        It then starts the threads.

        Args:
            None

        Returns:
            None
        """
        self.capture_event = threading.Event()
        self.publisher_event = threading.Event()

        self.capturer_thread = threading.Thread(target=self.record_audio)
        self.publisher_thread = threading.Thread(target=self.publish_audio)

        self.capturer_thread.start()
        self.publisher_thread.start()

    def stop_workers(self):
        """
        Stops the capture and publisher workers.

        This method sets the capture and publisher events to signal the workers to stop.
        It also waits for the workers to finish using the `join()` method.

        Args:
            None

        Returns:
            None
        """
        if hasattr(self, 'capture_event') and self.capture_event != None:
            self.capture_event.set()
        if hasattr(self, 'publisher_event') and self.publisher_event != None:
            self.publisher_event.set()
        if hasattr(self, 'publisher_thread') and self.publisher_thread.is_alive():
            self.publisher_thread.join()
        if hasattr(self, 'capturer_thread') and self.capturer_thread.is_alive():
            self.capturer_thread.join()
        
    def set_recording(self, request: SetState.Request, response: SetState.Response) -> SetState.Response:
        """
        Sets the desired state for node when request is recieved.

        Args:
            request: SetState.Request object - input to the service
            response: SetState.Response object - output to the service - Is empty when passed as argument
        
        Returns:
            SetState.Response object - response to the service
        """

        try:
            if request.desired_state == "active":
                self.start_workers()
                self.get_logger().info("MIC node: Recording started, node is active.")
            elif request.desired_state == "idle":
                self.stop_workers()
                self.get_logger().info("MIC node: Recording stopped, node is idle.")
            response.message = f"Successfully set state to: {request.desired_state}"
            response.success = True
        except Exception as e:
            response.message = f"Error setting state: {request.desired_state} - {e}"
            response.success = False
        return response
    
    def record_audio(self):
        """
        Records audio from the microphone.

        This method continuously captures audio data from the microphone and puts it into the capture queue.

        Args:
            None

        Returns:
            None
        """
        sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        channels = self.get_parameter('channels').get_parameter_value().integer_value
        duration = self.get_parameter('duration').get_parameter_value().double_value
        self.get_logger().info("MIC node: Recording audio with sample rate: " + str(sample_rate) + ", channels: " + str(channels) + ", duration: " + str(duration))
        while not self.capture_event.is_set():
            self.get_logger().info("MIC node: Starting the recording")
            audio_data = sd.rec(int(sample_rate * duration), samplerate=sample_rate, channels=channels, dtype=np.float32)
            sd.wait()
            self.get_logger().info("MIC node: Pausing the recording")
            msg = self._to_msg(audio_data, sample_rate, channels)            
            self.frame_num += 1
            self.capture_queue.put(msg)
            if self.get_parameter('capture_once').get_parameter_value().bool_value:
                self.get_logger().info("MIC node: Capture once is activated, stopping the recording")
                self.capture_event.set()

    def _to_msg(self, data, sample_rate, channels):
        """
        Converts audio data to a ROS2 message.

        This method converts the audio data, sample rate, channels, and duration into a ROS2 message format.

        Args:
            data: numpy array of audio data
            sample_rate: sample rate of the audio data
            channels: number of channels in the audio data

        Returns:
            Message of AudioData type
        """
        msg = AudioData()
        msg.data = data.tobytes()
        msg.sample_rate = sample_rate
        msg.channels = channels
        msg.samples = len(data)
        msg.header.frame_id = str(self.frame_num)
        return msg

    def publish_audio(self):
        """
        Publishes audio data.

        This method continuously publishes audio data from the capture queue.

        Args:
            None

        Returns:
            None
        """
        while True:
            if self.publisher_event.is_set():
                break
            try:
                msg = self.capture_queue.get(block=True, timeout=2)
            except queue.Empty:
                msg = None
            if self.publisher_event.is_set():
                break
            if msg != None:
                self.get_logger().info('MIC node: Publishing message to topic')
                print("Publishing a message \n")
                self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    micNode = AudioTransmitterNode()

    try:
        rclpy.spin(micNode)
    except KeyboardInterrupt:
        micNode.get_logger().info('MIC node: Keyboard interrupt')

    micNode.stop_workers()

    micNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()