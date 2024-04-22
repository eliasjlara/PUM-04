import queue
import threading
import time
import numpy as np
import subprocess
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from aida_interfaces.srv import SetState
from aida_interfaces.msg import Joystick

# from lidar_data.msg import LiDAR
import socket
import struct
import threading
import cv2

# Socket Constants
HEADER_FORMAT = "!HI"
MESSAGE_HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
VIDEO_STREAM_ID = 0  # We'll keep things simple with a single stream


# ROS2 Constants
VIDEO_TOPIC = "image"
LIDAR_TOPIC = "lidar"
STT_TOPIC = "stt/stt_result"
JOYSTICK_TOPIC = "joystick/pos"

CAMERA_CONTROL_SERVICE = "camera/SetState"
MIC_CONTROL_SERVICE = "mic/SetState"

STREAM_FREQUENCY = 30


# Message Type Enums (make sure these match your client-side definitions)
class MessageType:
    CAMERA = 1
    IMAGE_ANALYSIS = 2
    MIC = 3  # Not used
    STT = 4
    LIDAR = 5

    REQ_VIDEO_FEED = 6
    REQ_STT = 7
    REQ_LIDAR = 8

    TEXT = 9
    VIDEO_FRAME = 10
    LIDAR_DATA = 11
    AUDIO = 12
    JOYSTICK_MOVE = 14


msg_formats = {
    MessageType.CAMERA: "!H",
    MessageType.IMAGE_ANALYSIS: "!H",
    MessageType.MIC: "!H",
    MessageType.STT: "!H",
    MessageType.LIDAR: "!H",
    MessageType.REQ_VIDEO_FEED: "!H",
    MessageType.REQ_STT: "!H",
    MessageType.TEXT: "!H",
    MessageType.VIDEO_FRAME: "!HII",
    MessageType.LIDAR_DATA: "!H",
    MessageType.AUDIO: "!H",
    MessageType.JOYSTICK_MOVE: "!ii",
}


class Instruction:
    ON = 1
    OFF = 2


class InterfaceNode(Node):
    """
    A ROS2 node for communicating with AIDA.
    """

    def __init__(self, start_workers=True, host="localhost", port=6660):
        super().__init__("api_node")

        self.server_up = False
        self.bridge = CvBridge()
        self.host = host
        self.port = port

        self.init_clients()
        self.init_pubs()
        self.init_subs()
        self.init_queues()
        if start_workers:
            self.start_workers()

    def start_camera(self):
        # Start up a standard ros camera node
        print("Starting camera...")
        # Execute the command. Temporary and to be replaced with the actual ROS2 node for camera.
        subprocess.Popen(
            ["ros2", "run", "image_tools", "cam2image"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print("Camera started")

    def start_microphone(self):
        print("Starting microphone...")
        req = SetState.Request()
        req.desired_state = "active"
        self.future = self.mic_client.call_async(req)
        # rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())

    def stop_camera(self):
        req = SetState.Request()
        req.desired_state = "idle"
        self.future = self.camera_client.call_async(req)
        # rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())

    def stop_microphone(self):
        req = SetState.Request()
        req.desired_state = "idle"
        self.future = self.mic_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())

    def video_callback(self, msg) -> None:
        # print("Received video message")
        # image = np.frombuffer(msg.data, dtype=np.uint8)
        # self.video_queue_lock.acquire()
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.video_queue.put(cv_image)
        # print("Video frame added to queue: ", self.video_queue.qsize())
        # self.video_queue_lock.release()

    # def lidar_callback(self, msg) -> None:
    #     lidar_data = np.frombuffer(msg.data, dtype=np.uint8)
    #     self.lidar_queue_lock.acquire()
    #     self.lidar_queue.put(lidar_data)
    #     self.lidar_queue_lock.release()

    def stt_callback(self, msg) -> None:
        print("Received STT message:", msg.data)
        self.stt_queue_lock.acquire()
        self.stt_queue.put(msg.data)
        self.stt_queue_lock.release()

    def destroy_node(self):
        super().destroy_node()

    def init_clients(self) -> None:
        self.camera_client = self.create_client(SetState, CAMERA_CONTROL_SERVICE)
        self.mic_client = self.create_client(SetState, MIC_CONTROL_SERVICE)
        # self.lidar_client = self.create_client(SetState, 'lidar/SetState')

    def init_pubs(self):
        self.joystick_pub = self.create_publisher(Joystick, JOYSTICK_TOPIC, 10)

    def init_subs(self) -> None:
        self.video_sub = self.create_subscription(
            Image, VIDEO_TOPIC, self.video_callback, 10
        )
        self.stt_sub = self.create_subscription(
            String, STT_TOPIC, self.stt_callback, 10
        )
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
        self.joystick_queue_lock = threading.Lock()
        self.joystick_queue = queue.Queue()

    def start_workers(self, start_socket=True) -> None:

        self.joystick_publisher_event = threading.Event()
        self.joystick_publisher_event.clear()
        self.joystick_publisher_thread = threading.Thread(
            target=self.publish_joystick, name="joystick_publisher"
        )
        self.joystick_publisher_thread.start()

        if start_socket:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.bind((self.host, self.port))
            self.server_event = threading.Event()
            self.server_event.clear()
            self.server_thread = threading.Thread(
                target=self.start_server, name="server_thread"
            )
            self.server_thread.start()

    def stop_workers(self):

        if (
            hasattr(self, "joystick_publisher_event")
            and self.joystick_publisher_event != None
        ):
            self.joystick_publisher_event.set()
        if (
            hasattr(self, "joystick_publisher_thread")
            and self.joystick_publisher_thread.is_alive()
        ):
            self.joystick_publisher_thread.join()

        if hasattr(self, "server_event") and self.server_event != None:
            self.server_event.set()
        if hasattr(self, "server_thread") and self.server_thread.is_alive():
            self.server_thread.join()

    def _to_joystick_msg(self, data):
        """
        Converts joystick data to a ROS2 compatible Joystick message.

        This method packs the joystick x and y into the message format.
        """
        msg = Joystick()
        msg.x = data[0].tobytes()
        msg.y = data[1].tobytes()
        return msg

    def publish_joystick(self):
        """
        Publishes joystick data.

        This method continuously publishes joystick data from the joystick queue.
        """
        while True:
            if self.joystick_publisher_event.is_set():
                break
            try:
                msg = self.joystick_queue.get(block=True, timeout=2)
            except queue.Empty:
                msg = None
            if self.joystick_publisher_event.is_set():
                break
            if msg != None:
                self.joystick_publisher.publish(msg)

    # def get_lidar(self):
    #     self.lidar_queue_lock.acquire()
    #     data = self.lidar_queue.get()
    #     self.lidar_queue_lock.release()
    #     return data

    def start_server(self):
        self.socket.listen()
        print("Server listening...")
        while not self.server_event.is_set():
            client, addr = self.socket.accept()
            print(f"Connected by {addr}")
            thread = threading.Thread(target=self.handle_client, args=(client, addr))
            thread.start()
        self.socket.close()

    def handle_client(self, client, addr):
        while not self.server_event.is_set():
            try:
                data = client.recv(MESSAGE_HEADER_SIZE)
                if not data:
                    break
                message_type, payload_length = struct.unpack(HEADER_FORMAT, data)
                if payload_length > 0:
                    payload = client.recv(payload_length)
                else:
                    # We do not require to receive a payload if the length is zero.
                    payload = b"\0x00"
                print(f"Received message type: {message_type}")
                self.handle_message(client, message_type, payload)
            except ConnectionError:
                print(f"Client {addr} disconnected")
                break

    def handle_message(self, client, message_type, data):
        if message_type == MessageType.CAMERA:
            instr = struct.unpack(msg_formats.get(MessageType.CAMERA), data)[0]
            self.handle_camera(instr)
        elif message_type == MessageType.IMAGE_ANALYSIS:
            self.handle_image_analysis(data)
        elif message_type == MessageType.MIC:
            instr = struct.unpack(msg_formats.get(MessageType.MIC), data)[0]
            self.handle_mic(instr)
        elif message_type == MessageType.STT:
            self.handle_stt(data)
        elif message_type == MessageType.LIDAR:
            self.handle_lidar(data)
        elif message_type == MessageType.REQ_VIDEO_FEED:
            self.handle_req_video_feed(client)
        elif message_type == MessageType.REQ_STT:
            self.handle_req_stt(client)
        elif message_type == MessageType.TEXT:
            self.handle_text(data)
        elif message_type == MessageType.JOYSTICK_MOVE:
            self.handle_joystick_move(data)

    def handle_camera(self, data):
        if data == Instruction.ON:
            self.start_camera()
        elif data == Instruction.OFF:
            self.stop_camera()
        else:
            print("Unknown camera instruction:", data)

    def handle_image_analysis(self, data):
        # Implement logic to handle image analysis message
        pass

    def handle_mic(self, data):
        if data == Instruction.ON:
            self.start_microphone()
        elif data == Instruction.OFF:
            self.stop_microphone()  # You will probably not do this because the mic is always on a certain duration
        else:
            print("Unknown mic instruction:", data)

    def handle_stt(self, data):
        # Implement logic to handle STT message
        pass

    def handle_lidar(self, data):
        # Implement logic to handle lidar message
        pass

    def handle_req_video_feed(self, client):
        # Send video stream
        thread = threading.Thread(target=self.send_video_stream, args=(client,))
        thread.start()

    def handle_req_stt(self, client):
        print("Getting STT message from queue...")
        self.stt_queue_lock.acquire()
        try:
            stt_res = self.stt_queue.get(block=False)
        except queue.Empty:
            stt_res = ""
        self.stt_queue_lock.release()
        stt_res = stt_res.encode("utf-8")

        print(f"Sending STT result: {stt_res}")
        # Send STT response
        client.sendall(struct.pack(HEADER_FORMAT, MessageType.TEXT, len(stt_res)))
        client.sendall(stt_res)

    def handle_text(self, text):
        # Echo text message to console
        print("Client says:", text)

    def handle_joystick_move(self, data):
        data = struct.unpack(msg_formats.get(MessageType.JOYSTICK_MOVE), data)
        jstk_msg = self._to_joystick_msg(data)
        self.joystick_queue_lock.acquire()
        self.joystick_queue.put(jstk_msg)
        self.joystick_queue_lock.release()

    def send_video_stream(self, conn):
        while True:  # Video streaming loop
            time.sleep(1 / STREAM_FREQUENCY)
            self.video_queue_lock.acquire()
            try:
                frame = self.video_queue.get(block=False)
            except queue.Empty:
                frame = None
            self.video_queue_lock.release()
            if frame is None:
                print("Video queue empty. Requesting too frequently or feed offline.")
            else:
                # cv_image = cv2.resize(frame, (960, 540))
                # print("Forwarding video frame from queue...")
                self.send_video_frame(conn, frame)

    def send_video_frame(self, conn, frame):
        frame_bytes = cv2.imencode(".jpg", frame)[1].tobytes()  # Encode as JPEG
        # Send video frame header
        conn.sendall(
            struct.pack(HEADER_FORMAT, MessageType.VIDEO_FRAME, len(frame_bytes))
        )
        # Send video frame data
        conn.sendall(frame_bytes)


def main(args=None):
    rclpy.init(args=args)

    api = InterfaceNode()

    try:
        rclpy.spin(api)
    except KeyboardInterrupt:
        api.get_logger().info("Keyboard interrupt")
    finally:
        api.stop_workers()
        api.server.shutdown()
        api.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
