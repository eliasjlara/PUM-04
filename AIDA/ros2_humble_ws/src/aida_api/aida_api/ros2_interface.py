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
import socket
import struct
import threading
import cv2

# from lidar_data.msg import LiDAR

# Socket Constants
HEADER_FORMAT = "!HI"
MESSAGE_HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
VIDEO_STREAM_ID = 0  # We'll keep things simple with a single stream


# ROS2 Constants
# VIDEO_TOPIC = "image"
VIDEO_TOPIC = "video_analysis/result"  # "video/camera"
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
    GESTURE = 3
    POSE = 4


class InterfaceNode(Node):
    """
    A ROS2 node for communicating with AIDA.

    This node provides an interface for communicating with AIDA.
    It allows starting and stopping various components such as the camera, microphone, gesture recognition, and pose recognition.
    It also handles video and speech-to-text (STT) callbacks, and provides methods for its internal initializing clients, publishers, subscribers, and queues.
    Additionally, it supports starting worker threads for publishing joystick data and running a socket server.

    Args:
        host (str): The host IP address.
        port (int): The port number.
    """

    def __init__(self, host="localhost", port=6662):
        """
        Initialize the InterfaceNode.

        Args:
            host (str): The host IP address.
            port (int): The port number.
        """
        super().__init__("api_node")

        self.bridge = CvBridge()
        self.video_frame = None
        self.stt_result = None
        self.video_frame_lock = threading.Lock()
        self.stt_result_lock = threading.Lock()
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.host, self.port))

        self.init_clients()
        self.init_pubs()
        self.init_subs()
        self.init_queues()

    def start_camera(self):
            """
            Start the camera.

            This method starts the camera by sending a request to set the desired state to "active".
            It logs a message indicating that the camera is being started and prints the result of the request.
            """
            self.get_logger().info("Server| Starting camera...")
            req = SetState.Request()
            req.desired_state = "active"
            self.future = self.camera_client.call_async(req)
            return self.future.result()

    def start_microphone(self):
        """
        Start the microphone.

        This method starts the microphone by sending a request to set the desired state to "active".
        It logs a message indicating that the microphone is being started and prints the result of the request.
        """
        self.get_logger().info("Server| Starting microphone...")
        req = SetState.Request()
        req.desired_state = "active"
        self.future = self.mic_client.call_async(req)
        print(self.future.result())

    def start_gesture_recognition(self):
        """
        Start gesture recognition.

        This method starts the gesture recognition by sending a request to set the desired state to "gesture".
        It logs a message indicating that the gesture recognition is being started and prints the result of the request.
        """
        self.get_logger().info("Server| Starting gesture recognition...")
        req = SetState.Request()
        req.desired_state = "gesture"
        self.future = self.image_analysis_client.call_async(req)
        print(self.future.result())

    def start_pose_recognition(self):
        """
        Start pose recognition.

        This method starts the pose recognition by sending a request to set the desired state to "pose".
        It logs a message indicating that the pose recognition is being started and prints the result of the request.
        """
        self.get_logger().info("Server| Starting pose recognition...")
        req = SetState.Request()
        req.desired_state = "pose"
        self.future = self.image_analysis_client.call_async(req)
        print(self.future.result())

    def stop_camera(self):
        """
        Stop the camera.

        This method stops the camera by sending a request to set the desired state to "idle".
        It logs a message indicating that the camera is being stopped and prints the result of the request.
        """
        self.get_logger().info("Server| Stopping camera...")
        req = SetState.Request()
        req.desired_state = "idle"
        self.future = self.camera_client.call_async(req)
        print(self.future.result())

    def stop_microphone(self):
        """
        Stop the microphone.

        This method stops the microphone by sending a request to set the desired state to "idle".
        It logs a message indicating that the microphone is being stopped and prints the result of the request.
        """
        self.get_logger().info("Server| Stopping microphone...")
        req = SetState.Request()
        req.desired_state = "idle"
        self.future = self.mic_client.call_async(req)
        print(self.future.result())

    def stop_image_recognition(self):
        """
        Stop image recognition.

        This method stops the image recognition by sending a request to set the desired state to "idle".
        It logs a message indicating that the image recognition is being stopped and prints the result of the request.
        """
        self.get_logger().info("Server| Stopping image recognition...")
        req = SetState.Request()
        req.desired_state = "idle"
        self.future = self.image_analysis_client.call_async(req)
        print(self.future.result())

    def video_callback(self, msg) -> None:
        """
        Callback function for video messages.

        Args:
            msg: The video message.
        """
        self.video_frame_lock.acquire()
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.video_frame = cv_image
        self.video_frame_lock.release()

    def stt_callback(self, msg) -> None:
        """
        Callback function for STT messages.

        Args:
            msg: The STT message.
        """
        self.get_logger().info("Received STT message:", msg.data)
        self.stt_result_lock.acquire()
        self.stt_result = msg.data
        self.stt_result_lock.release()

    def destroy_node(self):
        """
        Destroy the InterfaceNode.
        """
        super().destroy_node()

    def init_clients(self) -> None:
        """
        Initialize the clients.

        This method initializes the clients for camera and microphone control.
        """
        self.camera_client = self.create_client(SetState, CAMERA_CONTROL_SERVICE)
        self.mic_client = self.create_client(SetState, MIC_CONTROL_SERVICE)

    def init_pubs(self):
        """
        Initialize the publishers.

        This method initializes the publisher for joystick data.
        """
        self.joystick_pub = self.create_publisher(Joystick, JOYSTICK_TOPIC, 10)

    def init_subs(self) -> None:
        """
        Initialize the subscribers.

        This method initializes the subscribers for video and speech-to-text (STT) messages.
        """
        self.video_sub = self.create_subscription(
            Image, VIDEO_TOPIC, self.video_callback, 10
        )
        self.stt_sub = self.create_subscription(
            String, STT_TOPIC, self.stt_callback, 10
        )

    def init_queues(self):
        """
        Initialize the queues.

        This method initializes the queue for joystick data.
        """
        self.joystick_queue_lock = threading.Lock()
        self.joystick_queue = queue.Queue()

    def start_workers(self, start_socket=True) -> None:
        """
        Start the worker threads.

        Args:
            start_socket (bool): Whether to start the socket server.
        """
        self.joystick_publisher_event = threading.Event()
        self.joystick_publisher_event.clear()
        self.joystick_publisher_thread = threading.Thread(
            target=self.publish_joystick, name="joystick_publisher"
        )
        self.joystick_publisher_thread.start()

        if start_socket:
            self.server_event = threading.Event()
            self.server_event.clear()
            self.server_thread = threading.Thread(
                target=self.start_server, name="server_thread"
            )
            self.server_thread.start()

    def stop_workers(self):
        """
        Stop the worker threads.
        """
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
        Convert joystick data to a ROS2 compatible Joystick message.

        Args:
            data: The joystick data.

        Returns:
            Joystick: The converted Joystick message.
        """
        msg = Joystick()
        msg.x = data[0].tobytes()
        msg.y = data[1].tobytes()
        return msg

    def publish_joystick(self):
        """
        Publish joystick data.

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

    def start_server(self):
            """
            Start the socket server and listen for incoming connections.

            This method starts the socket server and listens for incoming connections on the specified host and port.
            It accepts client connections and spawns a new thread to handle each client connection.
            The server continues to listen for connections until the server event is set.
            """
            self.socket.listen()
            self.get_logger().info(f"Server| Listening on {self.host}:{self.port}")
            while not self.server_event.is_set():
                client, addr = self.socket.accept()
                self.get_logger().info(f"Server| Connection from {addr}")
                thread = threading.Thread(target=self.handle_client, args=(client, addr))
                thread.start()
            self.socket.close()

    def handle_client(self, client, addr):
        """
        Handle a client connection.

        This method receives messages from a client and processes them accordingly.
        It continuously listens for incoming messages until the client connection is closed.
        The messages are composed of a header and a payload, where the header contains the message type and the payload length.
        The header and payload are sent separately, with the header being sent first.
        
        Args:
            client: The client socket.
            addr: The client address.
        """
        client_connected = True
        while client_connected:
            try:
                data = client.recv(MESSAGE_HEADER_SIZE)
                if not data:
                    client_connected = False
                    self.get_logger().info(f"Server| Connection to [{addr}] was closed.")
                    break
                message_type, payload_length = struct.unpack(HEADER_FORMAT, data)
                if payload_length > 0:
                    payload = client.recv(payload_length)
                else:
                    # We do not require to receive a payload if the length is zero.
                    payload = b"\0x00"
                self.get_logger().info(f"Server| Received message {message_type} from {addr}")
                self.handle_message(client, message_type, payload)
            except ConnectionError:
                self.get_logger().info(f"Server| Connection to [{addr}] was interrupted.")
                break

    def handle_message(self, client, message_type, data):
        """
        Handle a received message.

        This method is responsible for handling different types of messages received by the server.
        It takes the client socket, message type, and message data as arguments and passes them to the appropriate handler method based on the message type.

        Args:
            client: The client socket.
            message_type: The type of the received message.
            data: The data associated with the received message.
        """
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
        else:
            self.get_logger().info(f"Server| Unknown message type: {message_type}")

    def handle_camera(self, data):
        """
        Handle camera instructions.

        Turns the camera on or off based on the received instruction.

        Args:
            data: The camera instruction.
        """
        if data == Instruction.ON:
            self.start_camera()
        elif data == Instruction.OFF:
            self.stop_camera()
        else:
            self.get_logger().info("Unknown camera instruction:", data)

    def handle_image_analysis(self, data):
        """
        Handle image analysis instructions.

        Either starts gesture recognition, pose recognition, or stops image recognition based on the received instruction.

        Args:
            data: The image analysis instruction.
        """
        if data == Instruction.GESTURE:
            self.start_gesture_recognition()
        elif data == Instruction.POSE:
            self.start_pose_recognition()
        elif data == Instruction.OFF:
            self.stop_image_recognition()
        else:
            self.get_logger().info("Unknown image analysis instruction:", data)

    def handle_mic(self, data):
        """
        Handle microphone instructions.

        
        Turns the microphone on or off based on the received instruction.
        Args:
            data: The microphone instruction.
        """
        if data == Instruction.ON:
            self.start_microphone()
        elif data == Instruction.OFF:
            self.stop_microphone()
        else:
            self.get_logger().info("Unknown mic instruction:", data)

    def handle_stt(self, data):
        # Implement logic to handle STT message
        pass

    def handle_lidar(self, data):
        # Implement logic to handle lidar message
        pass

    def handle_req_video_feed(self, client):
        """
        Handle requests for video feed.

        
        Starts a new thread to send the video feed to the client.
        Args:
            client: The client socket.
        """
        self.get_logger().info("Server| Sending video feed to client.")
        # Send video stream
        thread = threading.Thread(target=self.send_video_stream, args=(client,))
        thread.start()

    def handle_req_stt(self, client):
        """
        Handle requests for STT.

        Send the latest STT result to the client.
        Args:
            client: The client socket.
        """
        self.stt_result_lock.acquire()
        stt_res = self.stt_result
        self.stt_result_lock.release()
        stt_res = stt_res.encode("utf-8")
        # Send STT response
        client.sendall(struct.pack(HEADER_FORMAT, MessageType.TEXT, len(stt_res)))
        client.sendall(stt_res)

    def handle_text(self, text):
        """
        Handle text messages.

        Log the received text message to the console.
        Args:
            text: The text message.
        """
        # Echo text message to console
        self.get_logger().info(f"Server| Received text message: {text}")

    def handle_joystick_move(self, data):
        """
        Handle joystick movements.

        Convert the joystick movement data to a Joystick message and add it to the joystick queue.
        Args:
            data: The joystick movement data.
        """
        data = struct.unpack(msg_formats.get(MessageType.JOYSTICK_MOVE), data)
        jstk_msg = self._to_joystick_msg(data)
        self.joystick_queue_lock.acquire()
        self.joystick_queue.put(jstk_msg)
        self.joystick_queue_lock.release()

    def send_video_stream(self, conn):
        """
        Send video stream to a client.

        This method sends the video stream to a client by continuously sending video frames at a specified frequency.
        Args:
            conn: The client socket.
        """
        while True:  # Video streaming loop
            time.sleep(1 / STREAM_FREQUENCY)
            self.video_frame_lock.acquire()
            frame = self.video_frame
            self.video_frame_lock.release()
            if frame is None:
                self.get_logger().info("Server| Video feed is empty.")
            else:
                try:
                    self.send_video_frame(conn, frame)
                except ConnectionError:
                    self.get_logger().info(f"Server| Video feed connection was interrupted.")
                    break

    def send_video_frame(self, conn, frame):
        """
        Send a video frame to a client.

        This method sends a video frame to a client by encoding the frame as a JPEG image and sending it over the socket connection.
        Args:
            conn: The client socket.
            frame: The video frame.
        """
        frame_bytes = cv2.imencode(".jpg", frame)[1].tobytes()  # Encode as JPEG
        # Send video frame header
        conn.sendall(
            struct.pack(HEADER_FORMAT, MessageType.VIDEO_FRAME, len(frame_bytes))
        )
        # Send video frame data
        conn.sendall(frame_bytes)


def main(args=None):
    rclpy.init(args=args)

    api = InterfaceNode(host="0.0.0.0")
    api.start_workers()
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
