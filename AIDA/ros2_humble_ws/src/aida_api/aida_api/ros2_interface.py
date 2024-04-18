import queue
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.sensor_msgs.msg import Image
from rclpy.sensor_msgs.msg import CompressedImage
from aida_interfaces.srv import SetState
from lidar_data.msg import LiDAR


VIDEO_TOPIC = '/camera/image_raw'
LIDAR_TOPIC = '/lidar'
STT_TOPIC = '/stt'


class InterfaceNode(Node):
    """
    A ROS2 node for communicating with AIDA.
    """

    def __init__(self):
        """
        Initializes the InterfaceNode.

        This method sets up the ROS2 node, initializes the publisher, and sets up the capture queue.
        """
        super().__init__('aida_api_node')

        self.init_clients()
        self.init_pubs()
        self.init_subs()
        self.init_queues()
        

    def video_callback(self, msg) -> None:
        """
        The callback function for processing the received video data.

        Args:
            msg: An img object representing the received video data.

        Returns:
            None
        """
        image = np.frombuffer(msg.data, dtype=np.uint8)
        self.video_queue_lock.acquire()
        self.video_queue.put(image)
        self.video_queue_lock.release()

    def lidar_callback(self, msg) -> None:
        lidar_data = np.frombuffer(msg.data, dtype=np.uint8)
        self.lidar_queue_lock.acquire()
        self.lidar_queue.put(lidar_data)
        self.lidar_queue_lock.release()

    def tts_callback(self, msg) -> None:
        stt_data = np.frombuffer(msg.data, dtype=np.uint8)
        self.stt_queue_lock.acquire()
        self.stt_queue.put(stt_data)
        self.stt_queue_lock.release()

    def destroy_node(self):
        """
        Destroys the AudioTransmitterNode.

        This method cleans up and destroys the ROS2 node.
        """
        super().destroy_node()

    def init_clients(self) -> None:
        self.camera_client = self.create_client(SetState, 'camera/SetState')
        self.lidar_client = self.create_client(SetState, 'lidar/SetState')
        self.mic_client = self.create_client(SetState, 'mic/SetState')



    def init_pubs(self):
        """
        Initializes the publisher.

        This method creates a publisher object for publishing audio data.
        """
        self.joystick = self.create_publisher(Joystick, self.get_parameter('joystick').get_parameter_value().string_value, 10)


    def init_subs(self) -> None:
        self.video_sub = self.create_subscription(
            Image,
            VIDEO_TOPIC,
            self.video_callback,
            10)
        self.stt_sub = self.create_subscription(
            String,
            STT_TOPIC,
            self.stt_callback,
            10)
        self.lidar_sub = self.create_subscription(
            LiDAR,
            LIDAR_TOPIC,
            self.lidar_callback,
            10)

    def init_queues(self):
        """
        Initializes the queues.

        This method sets up the queues and locks.
        """
        self.video_queue_lock = threading.Lock()
        self.video_queue = queue.Queue()
        self.lidar_queue_lock = threading.Lock()
        self.lidar_queue = queue.Queue()
        self.stt_queue_lock = threading.Lock()
        self.stt_queue = queue.Queue()
        self.joystick_queue_lock = threading.Lock()
        self.joystick_queue = queue.Queue()

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


    def _to_joystick_msg(self, data):
        """
        Converts audio data to a ROS2 message.

        This method converts the audio data, sample rate, channels, and duration into a ROS2 message format.
        """
        msg = Joystick()
        msg.data = data.tobytes()
        return msg

    def publish_joystick(self):
        """
        Publishes joystick data.

        This method continuously publishes joystick data from the joystick queue.
        """
        while True:
            if self.publisher_event.is_set():
                break
            try:
                msg = self.capture_queue.get(block=True, timeout=2)
            except queue.Empty:§
                msg = None
            if self.publisher_event.is_set():
                break
            if msg != None:
                self.joystick_publisher.publish(msg)

    def get_video_frame(self):
        self.video_queue_lock.acquire()
        frame = self.video_queue.get()
        self.video_queue_lock.release()
        return frame
    def get_lidar(self):
        self.lidar_queue_lock.acquire()
        data = self.lidar_queue.get()
        self.lidar_queue_lock.release()
        return data
    def get_stt(self):
        self.stt_queue_lock.acquire()
        data = self.stt_queue.get()
        self.stt_queue_lock.release()
        return data
    def set_joystick(self, jstk):
        self.joystick_queue_lock.acquire()
        # TODO: convert to jstk msg type here
        self.joystick_queue.put(jstk)
        self.joystick_queue_lock.release()



def main(args=None):
    rclpy.init(args=args)

    micNode = InterfaceNode()

    try:
        rclpy.spin(micNode)
    except KeyboardInterrupt:
        micNode.get_logger().info('MIC: Keyboard interrupt')

    micNode.stop_workers()

    micNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()