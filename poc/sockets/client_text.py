import cv2
import numpy as np
import socket
import struct
import time

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

    SUCCESS = 15
    FAILURE = 16
    HEADER = 17

class Instruction:
    ON = 1
    OFF = 2

HEADER_FORMAT = "!HI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
server_address = ('localhost', 9000)
client_socket.connect(server_address)

while True:
    # Receive frame data
    print('SENDING TEXT...')

    text = "Hello from the client side!"
    text = text.encode()
    client_socket.sendall(struct.pack("!HI", MessageType.TEXT, len(text)))
    client_socket.sendall(text)
    time.sleep(1)
# Close the connection
client_socket.close()