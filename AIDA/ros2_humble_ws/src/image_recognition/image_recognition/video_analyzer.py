import sys
import os

# Add the directory containing gesture_recognizer.py to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

import numpy as np

from gesture_recognizer import GestureRecognizer
from pose_landmarker import PoseLandmarker

class VideoTransmitNode(Node):
    def __init__(self):
        super().__init__('video_transmit')
        self.publisher = self.create_publisher(Image, 'processed_video', 10)
        self.subscriber = self.create_subscription(Image, 'video/camera', self.callback, 10)
        self.bridge = CvBridge()

        self.img_out = None
        self.cv2_img = None
        #self.pose_landmarker = PoseLandmarker()
        self.gesture_recognizer = GestureRecognizer()


    def callback(self, msg):
        # received video data
        # converting video data

        self.cv2_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        #self.img_out = self.pose_landmarker.apply_pose_landmarking(self.cv2_img)
        self.img_out = self.gesture_recognizer.apply_gesture_detection(self.cv2_img)

        self.publish_video()

    def publish_video(self):
        #region Test remove
        
        # Converts an OpenCV image to a ROS2 Image Message (docs ROS: sensor_msgs/Image.msg)
        msg = self.bridge.cv2_to_imgmsg(self.img_out, 'bgr8')
        # Publishes to the 'video' topic
        self.publisher.publish(msg)
        
        #endregion
        #frame = self.gesture_recognizer.apply_gesture_detection()   # TODO: remove when done
        # frame = self.pose_landmarker.apply_pose_landmarking(frame) # XXX


def main(args=None):
    rclpy.init(args=args)
    node = VideoTransmitNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Video Transmitter: Keyboard interrupt')

    # Call cleanup functions when Ctrl+C is pressed
    #node.pose_landmarker.detector.close() # XXX
    node.gesture_recognizer.recognizer.close()
    cv2.destroyAllWindows()
    # Node cleanup
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
