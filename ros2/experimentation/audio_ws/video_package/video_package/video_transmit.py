import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class VideoTransmitNode(Node):
    def __init__(self):
        super().__init__('video_transmit')
        self.publisher = self.create_publisher(Image, 'video', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_video)  # Adjust the rate as needed
        self.cap = cv2.VideoCapture('http://195.196.36.242/mjpg/video.mjpg')  # Replace with your video file or stream URL

    def publish_video(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoTransmitNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()