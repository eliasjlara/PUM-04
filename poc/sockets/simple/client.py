import cv2
import numpy as np
import socket


# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
server_address = ('localhost', 6000)
client_socket.connect(server_address)

# Receive video frames from the server
while True:
    # Receive frame size
    print('Receiving frame size...')
    frame_size = client_socket.recv(4)
    frame_size = int.from_bytes(frame_size, byteorder='big')
    client_socket.sendall(b'ack\n')
    # Receive frame data
    print('Receiving frame...')
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