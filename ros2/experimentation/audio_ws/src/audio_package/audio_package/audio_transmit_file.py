import queue
import threading

import rclpy
from rclpy.node import Node
from audio_data.msg import AudioData 
import numpy as np
import time
import librosa

class AudioTransmitterFileNode(Node):
    """
    A ROS2 node for transmitting audio data.

    This node initializes a publisher to publish audio data and sends data from a specific audio file
    It provides methods to start and stop the capture and publisher workers.
    """

    def __init__(self):
        """
        Initializes the AudioTransmitterFileNode.

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
        """
        super().destroy_node()

    def initialize_publisher(self):
        """
        Initializes the publisher.

        This method creates a publisher object for publishing audio data.
        """
        self.publisher = self.create_publisher(AudioData, self.get_parameter('audio_topic').get_parameter_value().string_value, 10)
        self.frame_num = 0

    def initialize_capture_queue(self):
        """
        Initializes the capture queue and workers.

        This method sets up the capture queue, lock, and threads for capturing and publishing audio data.
        """
        self.queue_lock = threading.Lock()

        self.capture_queue = queue.Queue()

        self.capture_event = threading.Event()
        self.capturer_thread = threading.Thread(target=self.get_audio_from_file, name='capturer')

        self.publisher_event = threading.Event()
        self.publisher_thread = threading.Thread(target=self.publish_audio, name='publisher')

        self.capturer_thread.start()
        self.publisher_thread.start()

    def stop_workers(self):
        """
        Stops the capture and publisher workers.

        This method sets the capture and publisher events to signal the workers to stop.
        It also waits for the workers to finish using the `join()` method.
        """
        if hasattr(self, 'capture_event') and self.capture_event != None:
            self.capture_event.set()
        if hasattr(self, 'publisher_event') and self.publisher_event != None:
            self.publisher_event.set()
        if hasattr(self, 'publisher_thread') and self.publisher_thread.is_alive():
            self.publisher_thread.join()
        if hasattr(self, 'capturer_thread') and self.capturer_thread.is_alive():
            self.capturer_thread.join()

    def get_audio_from_file(self):
        """
        This method converts and puts audio from file in capture queue to be sent to subscribers

        """
        sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        channels = self.get_parameter('channels').get_parameter_value().integer_value
        duration = self.get_parameter('duration').get_parameter_value().double_value
        
        audio_path = "/home/alex/projects/PUM-04/ros2/experimentation/audio_ws/src/audio_package/resource/Move3mForwardTurnRight2.wav"
        audio_data, _ = librosa.load(audio_path, sr=sample_rate)
        #audio_data = audio_data * (2**16)
        #audio_data = audio_data.astype(np.int32)
        msg = self._to_msg(audio_data, sample_rate=sample_rate, channels=channels)

        while not self.capture_event.is_set():
            self.capture_queue.put(msg)
            time.sleep(5)


    def _to_msg(self, data, sample_rate, channels):
        """
        Converts audio data to a ROS2 message.

        This method converts the audio data, sample rate, channels, and duration into a ROS2 message format.
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
                print("publishing message ")
                self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    micNode = AudioTransmitterFileNode()

    try:
        rclpy.spin(micNode)
    except KeyboardInterrupt:
        micNode.get_logger().info('MIC: Keyboard interrupt')

    micNode.stop_workers()

    micNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()