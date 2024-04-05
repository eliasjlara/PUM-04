import socket
import cv2

# Set up socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('localhost', 6000))
server_socket.listen(1)
print('Server listening on port 6000...')
try:
    # Open video capture
    video_capture = cv2.VideoCapture(0)
    while True:
        # Accept client connection
        client_socket, address = server_socket.accept()
        print('Client connected:', address)

        while True:
            # Read frame from video capture
            ret, frame = video_capture.read()

            # Convert frame to bytes
            _, frame_data = cv2.imencode('.jpg', frame)
            frame_bytes = frame_data.tobytes()

            # Send frame size to client
            frame_size = len(frame_bytes).to_bytes(4, byteorder='big')
            client_socket.sendall(frame_size)

            # Send frame to client
            print('Sending frame...')
            client_socket.sendall(frame_bytes)

            # Check if client wants to disconnect
            data = client_socket.recv(1024)
            if data.split(b"\n")[0] == b'disconnect':
                break

        # Close client connection
        client_socket.close()
        print('Client disconnected')
except KeyboardInterrupt:
    print('Server shutting down...')
finally:

        # Release video capture and close server socket
        video_capture.release()
        server_socket.close()