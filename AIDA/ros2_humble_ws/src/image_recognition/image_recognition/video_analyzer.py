import sys
import os

# Add the directory containing gesture_recognizer.py to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from aida_interfaces.srv import SetState
from sensor_msgs.msg import Image
import cv2

import numpy as np

from gesture_recognizer import GestureRecognizer
from pose_landmarker import PoseLandmarker

class AnalysisType:
    NONE = 1
    POSE_LANDMARKER = 2
    GESTURE_RECOGNIZER = 3


class VideoTransmitNode(Node):
    def __init__(self):
        super().__init__('video_analyzer')
        self.publisher = self.create_publisher(Image, 'video_analysis/result', 10)
        self.subscriber = self.create_subscription(Image, 'video/camera', self.callback, 10)
        self.srv = self.create_service(SetState, 'video_analyzer/SetState', self.set_state_callback)
        self.bridge = CvBridge()

        self.img_out = None
        self.cv2_img = None
        self.active_analysis = AnalysisType.GESTURE_RECOGNIZER
        # TODO: Don't load both models at the same time
        self.pose_landmarker = PoseLandmarker()
        self.gesture_recognizer = GestureRecognizer()


    def callback(self, msg):
        self.cv2_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Apply the selected analysis. Only one analysis can be active at a time.
        if self.active_analysis == AnalysisType.POSE_LANDMARKER:
            self.img_out = self.pose_landmarker.apply_pose_landmarking(self.cv2_img)
        elif self.active_analysis == AnalysisType.GESTURE_RECOGNIZER:
            self.img_out = self.gesture_recognizer.apply_gesture_detection(self.cv2_img)
        elif self.active_analysis == AnalysisType.NONE:
            self.img_out = self.cv2_img
        self.publish_frame()

    def publish_frame(self):        
        # Converts an OpenCV image to a ROS2 Image Message (docs ROS: sensor_msgs/Image.msg)
        msg = self.bridge.cv2_to_imgmsg(self.img_out, 'bgr8')
        # Publishes to the 'video' topic
        self.publisher.publish(msg)
        
    def set_state_callback(self, request, response):
        response.success = True
        if request.desired_state == "pose":
            self.active_analysis = AnalysisType.POSE_LANDMARKER
            response.message = " Pose landmarking activated."
        elif request.desired_state == "gesture":
            self.active_analysis = AnalysisType.GESTURE_RECOGNIZER
            response.message = "Gesture analysis activated. "
        elif request.desired_state == "idle":
            self.active_analysis = AnalysisType.NONE
            response.message = "All analysis deactivated."
        else:
            response.success = False
            response.message = "Invalid desired state. Options: 'pose', 'gesture', 'idle'"
        return response



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
