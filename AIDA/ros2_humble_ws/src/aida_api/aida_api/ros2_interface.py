import queue
import threading
import time
import numpy as np

from server import SocketServer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.sensor_msgs.msg import Image
from rclpy.sensor_msgs.msg import CompressedImage
from aida_interfaces.srv import SetState
# from lidar_data.msg import LiDAR


VIDEO_TOPIC = 'camera/image_raw'
LIDAR_TOPIC = 'lidar'
STT_TOPIC = 'stt/stt_result'
JOYSTICK_TOPIC = 'joystick/pos'


class InterfaceNode(Node):
    """
    A ROS2 node for communicating with AIDA.
    """

    def __init__(self):
        super().__init__('api_node')

        self.init_server()

        self.init_clients()
        # self.init_pubs()
        self.init_subs()
        self.init_queues()

    def start_camera(self):
        req = SetState.Request()
        req.state = "active"
        self.future = self.camera_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())

    def start_microphone(self):
        req = SetState.Request()
        req.state = "active"
        self.future = self.mic_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())

    def stop_camera(self):
        req = SetState.Request()
        req.state = "idle"
        self.future = self.camera_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())

    def stop_microphone(self):
        req = SetState.Request()
        req.state = "idle"
        self.future = self.mic_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())

    def video_callback(self, msg) -> None:
        image = np.frombuffer(msg.data, dtype=np.uint8)
        self.video_queue_lock.acquire()
        self.video_queue.put(image)
        self.video_queue_lock.release()

    # def lidar_callback(self, msg) -> None:
    #     lidar_data = np.frombuffer(msg.data, dtype=np.uint8)
    #     self.lidar_queue_lock.acquire()
    #     self.lidar_queue.put(lidar_data)
    #     self.lidar_queue_lock.release()

    def tts_callback(self, msg) -> None:
        stt_data = np.frombuffer(msg.data, dtype=np.uint8)
        self.stt_queue_lock.acquire()
        self.stt_queue.put(stt_data)
        self.stt_queue_lock.release()

    def destroy_node(self):
        super().destroy_node()

    def init_clients(self) -> None:
        self.camera_client = self.create_client(SetState, 'camera/SetState')
        self.mic_client = self.create_client(SetState, 'mic/SetState')
        # self.lidar_client = self.create_client(SetState, 'lidar/SetState')

    # def init_pubs(self):
    #     self.joystick = self.create_publisher(Joystick, JOYSTICK_TOPIC, 10)

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
        # self.lidar_sub = self.create_subscription(
        #     LiDAR,
        #     LIDAR_TOPIC,
        #     self.lidar_callback,
        #     10)

    def init_queues(self):
        self.video_queue_lock = threading.Lock()
        self.video_queue = queue.Queue()
        # self.lidar_queue_lock = threading.Lock()
        # self.lidar_queue = queue.Queue()
        self.stt_queue_lock = threading.Lock()
        self.stt_queue = queue.Queue()
        # self.joystick_queue_lock = threading.Lock()
        # self.joystick_queue = queue.Queue()

    # def start_workers(self) -> None:

        # self.joystick_publisher_event = threading.Event()
        # self.joystick_publisher_event.clear()
        # self.joystick_publisher_thread = threading.Thread(target=self.publish_joystick, name='joystick_publisher')

        # self.joystick_publisher_thread.start()


    # def stop_workers(self):

        # if hasattr(self, 'joystick_publisher_event') and self.joystick_publisher_event != None:
        #     self.joystick_publisher_event.set()
        # if hasattr(self, 'joystick_publisher_thread') and self.joystick_publisher_thread.is_alive():
        #     self.joystick_publisher_thread.join()


    # def _to_joystick_msg(self, data):
    #     """
    #     Converts audio data to a ROS2 message.

    #     This method converts the audio data, sample rate, channels, and duration into a ROS2 message format.
    #     """
    #     msg = Joystick()
    #     msg.x = data[0].tobytes()
    #     msg.y = data[1].tobytes()
    #     return msg

    # def publish_joystick(self):
    #     """
    #     Publishes joystick data.

    #     This method continuously publishes joystick data from the joystick queue.
    #     """
    #     while True:
    #         if self.joystick_publisher_event.is_set():
    #             break
    #         try:
    #             msg = self.capture_queue.get(block=True, timeout=2)
    #         except queue.Empty:
    #             msg = None
    #         if self.joystick_publisher_event.is_set():
    #             break
    #         if msg != None:
    #             self.joystick_publisher.publish(msg)

    def get_video_frame(self):
        self.video_queue_lock.acquire()
        frame = self.video_queue.get()
        self.video_queue_lock.release()
        return frame
    
    # def get_lidar(self):
    #     self.lidar_queue_lock.acquire()
    #     data = self.lidar_queue.get()
    #     self.lidar_queue_lock.release()
    #     return data
    
    def get_stt(self):
        self.stt_queue_lock.acquire()
        data = self.stt_queue.get()
        self.stt_queue_lock.release()
        return data
    
    # def set_joystick(self, jstk: list):
    #     jstk_msg = self._to_joystick_msg(jstk)
    #     self.joystick_queue_lock.acquire()
    #     self.joystick_queue.put(jstk_msg)
    #     self.joystick_queue_lock.release()

    def init_server(self):
        server = SocketServer('localhost', 9000, self)  # Adjust host and port as needed
        try:
            server.start()
        except KeyboardInterrupt:
            print('Server shutting down...')
        finally:
            server.shutdown()
            print('Server stopped')


def main(args=None):
    rclpy.init(args=args)

    api = InterfaceNode()

    try:
        rclpy.spin(api)
    except KeyboardInterrupt:
        api.get_logger().info('Keyboard interrupt')

    api.stop_workers()

    api.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()