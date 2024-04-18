import socket
import struct
import threading
import cv2

# Constants
HEADER_FORMAT = "!HI"
MESSAGE_HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
VIDEO_STREAM_ID = 0  # We'll keep things simple with a single stream

# Message Type Enums (make sure these match your client-side definitions)
class MessageType:
    CAMERA = 1
    IMAGE_ANALYSIS = 2
    MIC = 3 # Not used
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
    MessageType.CAMERA: '!H',
    MessageType.IMAGE_ANALYSIS: '!H',
    MessageType.MIC: '!H',
    MessageType.STT: '!H',
    MessageType.LIDAR: '!H',
    MessageType.REQ_VIDEO_FEED: '!H',
    MessageType.REQ_STT: '!H',
    MessageType.TEXT: '!H',
    MessageType.VIDEO_FRAME: '!HII',
    MessageType.LIDAR_DATA: '!H',
    MessageType.AUDIO: '!H',
    MessageType.JOYSTICK_MOVE: '!ii'
}



class Instruction:
    ON = 1
    OFF = 2


class SocketServer:
    def __init__(self, host, port, ros):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        self.io_threads = []
        self.ros = ros
        
        self.active_video = False
        self.active_analysis = False
        self.active_audio = False

        # Open video capture - to be replaced with ROS image topic
        self.video_capture = None

    def start(self):
        self.socket.listen()
        print('Server listening...')
        while True:
            client, addr = self.socket.accept()
            print(f'Connected by {addr}')
            thread = threading.Thread(target=self.handle_client, args=(client, addr))
            thread.start()

    def handle_client(self, client, addr):
        while True:
            try:
                data = client.recv(MESSAGE_HEADER_SIZE) 
                if not data: 
                    break
                message_type, payload_length = struct.unpack(HEADER_FORMAT, data)
                if payload_length > 0:
                    payload = client.recv(payload_length)
                else:
                    # We do not require to receive a payload if the length is zero.
                    payload = b'\0x00'
                self.handle_message(client, message_type, payload)
            except ConnectionError:
                print(f'Client {addr} disconnected')
                break

    def handle_message(self, client, message_type, data):
        if message_type == MessageType.CAMERA:
            instr = struct.unpack(msg_formats.get(MessageType.CAMERA), data)[0]
            # self.handle_camera(instr)
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
            self.ros.start_camera()
        elif data == Instruction.OFF:
            self.ros.stop_camera()
        else:
            print('Unknown camera instruction:', data)

    # def handle_camera(self, data):
    #     if data == Instruction.ON:
    #         self.video_capture = cv2.VideoCapture(0)
    #         self.active_video = True
    #         return self.video_capture.isOpened()
    #     elif data == Instruction.OFF:
    #         if self.video_capture:
    #             self.video_capture.release()
    #             self.video_capture = None
    #             self.active_video = False  
    #     else:
    #         print('Unknown camera instruction:', data)

    def handle_image_analysis(self, data):
        # Implement logic to handle image analysis message
        pass

    def handle_mic(self, data):
        # Implement logic to handle mic message
        if data == Instruction.ON:
            self.ros.start_microphone()
        elif data == Instruction.OFF:
            self.ros.stop_microphone() # You will probably not do this because the mic is always on a certain duration
        else:
            print('Unknown mic instruction:', data)

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
        self.io_threads.append(thread)

    def handle_req_stt(self, client):
        stt_res = self.ros.get_stt()
        if not stt_res:
            stt_res = '-- No speech detected --'
        
        stt_res = stt_res.encode('utf-8')
        
        print(f"Sending STT result: {stt_res}")
        # Send STT response
        client.sendall(struct.pack(HEADER_FORMAT, MessageType.TEXT, len(stt_res)))
        client.sendall(stt_res)

    def handle_text(self, text):
        # Echo text message to console
        print("Client says:", text)

    def handle_joystick_move(self, data):
        data = struct.unpack(msg_formats.get(MessageType.JOYSTICK_MOVE), data)
        self.ros.set_steering(data)

    def send_video_stream(self, conn):
        while True:  # Video streaming loop
            frame = self.get_video_frame()  # Get the next video frame

            if frame is None:
                break
            self.send_video_frame(conn, frame)

    def send_video_frame(self, conn, frame):
        frame_bytes = cv2.imencode(".jpg", frame)[1].tobytes()  # Encode as JPEG 
        frame_size = len(frame_bytes)
        # Send video frame header
        conn.sendall(struct.pack(HEADER_FORMAT, MessageType.VIDEO_FRAME, len(frame_bytes)))
        # Send video frame data
        conn.sendall(frame_bytes)

    def get_video_frame(self):
        return self.ros.get_video_frame()

    # def get_video_frame(self):
    #     # Capture a video frame
    #     if self.video_capture:
    #         ret, frame = self.video_capture.read()
    #     else:
    #         print('No video capture device available')
    #         frame = None
    #     return frame



    def shutdown(self):
        if self.socket:
            self.socket.close()

if __name__ == '__main__':
    server = SocketServer('localhost', 9000, None)  # Adjust host and port as needed
    try:
        server.start()
    except KeyboardInterrupt:
        print('Server shutting down...')
    finally:
        server.shutdown()
        print('Server stopped')