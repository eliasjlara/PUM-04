import queue
import threading
import sounddevice as sd

import rclpy
from rclpy.node import Node
from audio_data.msg import AudioData 
import numpy as np


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
        Initializes the capture queue and workers.

        This method sets up the capture queue, lock, and threads for capturing and publishing audio data.

        Args:
            None

        Returns:
            None
        """
        self.queue_lock = threading.Lock()

        self.capture_queue = queue.Queue()

        self.capture_event = threading.Event()
        self.capturer_thread = threading.Thread(target=self.record_audio, name='capturer')

        self.publisher_event = threading.Event()
        self.publisher_thread = threading.Thread(target=self.publish_audio, name='publisher')

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
        print("startning recording \ n")
        audio_data = sd.rec(int(sample_rate * duration), samplerate=sample_rate, channels=channels, dtype=np.float32)
        sd.wait()
        print("pausing the recording\n")
        msg = self._to_msg(audio_data, sample_rate, channels, duration)            
        self.frame_num += 1
        self.capture_queue.put(msg)

    def _to_msg(self, data, sample_rate, channels, duration):
        """
        Converts audio data to a ROS2 message.

        This method converts the audio data, sample rate, channels, and duration into a ROS2 message format.

        Args:
            None

        Returns:
            Message of AudioData type
        """
        msg = AudioData()
        msg.data = data.tobytes()
        msg.sample_rate = sample_rate
        msg.channels = channels
        msg.samples = int(sample_rate * duration)
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
                print("Publishing a message \n")
                self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    micNode = AudioTransmitterNode()

    try:
        rclpy.spin(micNode)
    except KeyboardInterrupt:
        micNode.get_logger().info('MIC: Keyboard interrupt')

    micNode.stop_workers()

    micNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()