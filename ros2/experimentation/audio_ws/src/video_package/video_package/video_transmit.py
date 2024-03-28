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
        self.timer = self.create_timer(
            0.1, self.publish_video)  # Adjust the rate as needed
        # Replace with your video file or stream URL
        self.cap = cv2.VideoCapture('http://195.196.36.242/mjpg/video.mjpg')

    def publish_video(self):
        ret, frame = self.cap.read()
        if ret:
            # Converts an OpenCV image to a ROS2 Image Message (docs ROS: sensor_msgs/Image.msg)
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            # Publishes to the 'video' topic
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VideoTransmitNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Video Transmitter: Keyboard interrupt')
    
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
