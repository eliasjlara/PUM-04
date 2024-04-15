import cv2
import numpy as np
import socket
import struct


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
    VIDEO_CONFIG = 13
    JOYSTICK_MOVE = 14

class Instruction:
    ON = 1
    OFF = 2

HEADER_FORMAT = "!HI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
server_address = ('localhost', 8000)
client_socket.connect(server_address)

length = struct.calcsize("!H")
client_socket.sendall(struct.pack("!HI", MessageType.CAMERA, length))
client_socket.sendall(struct.pack("!H", Instruction.ON))

client_socket.sendall(struct.pack("!HI", MessageType.REQ_VIDEO_FEED, 0))


# Receive video frames from the server
while True:
    # Receive frame data
    print('Receiving frame...')

    header = client_socket.recv(HEADER_SIZE)
    message_type, frame_size = struct.unpack(HEADER_FORMAT, header)

    # Recieve the frame
    # frame_data = client_socket.recv(frame_size)

    frame_data = b''
    while len(frame_data) < frame_size:
        remaining_bytes = frame_size - len(frame_data)
        frame_data += client_socket.recv(remaining_bytes)

    # Convert frame data to numpy array
    frame_array = np.frombuffer(frame_data, dtype=np.uint8)

    # Decode and display the frame
    frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
    cv2.imshow('Video Feed', frame)
    cv2.waitKey(1)

# Close the connection
client_socket.close()