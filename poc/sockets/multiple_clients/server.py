import socket
import cv2
import threading

def handle_client(conn, addr):
    while True:
        try:
            # Read frame from video capture
            ret, frame = video_capture.read()

            # Convert frame to bytes
            _, frame_data = cv2.imencode('.jpg', frame)
            frame_bytes = frame_data.tobytes()

            # Send frame size to client
            frame_size = len(frame_bytes).to_bytes(4, byteorder='big')
            print(conn)
            conn.sendall(frame_size)

            # Send frame to client
            print('Sending frame...')
            conn.sendall(frame_bytes)

            # Check if client wants to disconnect
            data = conn.recv(1024)
            if data.split(b"\n")[0] == b'disconnect':
                break

        except ConnectionError:
            print(f'Client {addr} disconnected')
            break

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
        print('Client connected:', address, client_socket)

        thread = threading.Thread(target=handle_client, args=(client_socket, address))
        thread.start()

except KeyboardInterrupt:
    print('Server shutting down...')
finally:

        # Release video capture and close server socket
        video_capture.release()
        server_socket.close()




