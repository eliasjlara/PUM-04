import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.message import 
from rclpy.sensor_msgs.msg import Image
from rclpy.sensor_msgs.msg import CompressedImage

class AudioTransmitterNode(Node):
    """
    A ROS2 node for transmitting audio data.

    This node initializes a publisher to publish audio data and captures audio from the microphone.
    It provides methods to start and stop the capture and publisher workers.
    """

    def __init__(self):
        """
        Initializes the AudioTransmitterNode.

        This method sets up the ROS2 node, initializes the publisher, and sets up the capture queue.
        """
        super().__init__('ros2_mic_node', namespace='mic')

        self.declare_parameter("audio_topic", "mic_audio")
        self.declare_parameter("sample_rate", 44100)
        self.declare_parameter("channels", 2)
        self.declare_parameter("duration", 5.0)
        self.declare_parameter("capture_once", True)

        self.init_services()
        self.init_publisher()
        self.init_capture_queue()
    
        super().__init__('audio_receiver_node', namespace='mic')
        self.subscription = self.create_subscription(
            Image,
            'mic_audio',
            self.listener_callback,
            10)
        self.subscription
        self.file_counter: int = 0

    def listener_callback(self, msg) -> None:
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

    def set_recording(self, request: SetState.Request, response: SetState.Response) -> SetState.Response:
        try:
            if request.desired_state == "active":
                self.start_workers()
                print("Recording started, node is active.")
            elif request.desired_state == "idle":
                self.stop_workers()
                print("Recording cancelled, node is idle.")
            response.message = f"Successfully set state to: {request.desired_state}"
            response.success = True
        except Exception as e:
            response.message = f"Error setting state: {request.desired_state} - {e}"
            response.success = False
        return response

    def destroy_node(self):
        """
        Destroys the AudioTransmitterNode.

        This method cleans up and destroys the ROS2 node.
        """
        super().destroy_node()

    def init_services(self) -> None:
        self.srv = self.create_service(SetState, 'SetState', self.set_recording)

    def init_publisher(self):
        """
        Initializes the publisher.

        This method creates a publisher object for publishing audio data.
        """
        self.publisher = self.create_publisher(AudioData, self.get_parameter('audio_topic').get_parameter_value().string_value, 10)
        self.frame_num = 0

    def init_capture_queue(self):
        """
        Initializes the capture queue and workers.

        This method sets up the capture queue, lock, and threads for capturing and publishing audio data.
        """
        self.queue_lock = threading.Lock()

        self.capture_queue = queue.Queue()

    def start_workers(self) -> None:
        self.capture_event = threading.Event()
        self.capture_event.clear()
        self.capturer_thread = threading.Thread(target=self.record_audio, name='capturer')

        self.publisher_event = threading.Event()
        self.publisher_event.clear()
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

    def record_audio(self):
        """
        Records audio from the microphone.

        This method captures audio data from the microphone and puts it into the capture queue.
        """
        sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        channels = self.get_parameter('channels').get_parameter_value().integer_value
        duration = self.get_parameter('duration').get_parameter_value().double_value
        print(f"Recording audio at {sample_rate} Hz, {channels} channels, for {duration} seconds")
        while not self.capture_event.is_set():
            print("Recording...")
            audio_data = sd.rec(int(sample_rate * duration), samplerate=sample_rate, channels=channels, dtype='uint8')
            sd.wait()
            msg = self._to_msg(audio_data, sample_rate, channels, duration)            
            self.frame_num += 1
            self.capture_queue.put(msg)
            if self.get_parameter('capture_once').get_parameter_value().bool_value:
                print("Capture once is activated, recording finished.")
                self.capture_event.set()

    def _to_msg(self, data, sample_rate, channels, duration):
        """
        Converts audio data to a ROS2 message.

        This method converts the audio data, sample rate, channels, and duration into a ROS2 message format.
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