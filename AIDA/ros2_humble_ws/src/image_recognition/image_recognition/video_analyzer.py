import sys
import os
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from aida_interfaces.srv import SetState
from sensor_msgs.msg import Image
import cv2
import numpy as np
from gesture_recognizer import GestureRecognizer
from pose_landmarker import PoseLandmarker

# Add the directory containing gesture_recognizer.py to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

class AnalysisType:
    NONE = 1
    POSE_LANDMARKER = 2
    GESTURE_RECOGNIZER = 3


class VideoTransmitNode(Node):
    """
    Node for analyzing video frames and publishing the results.

    Subscribes to the 'video/camera' topic for incoming video frames and publishes the analyzed frames to the 'video_analysis/result' topic.
    Provides a service 'video_analyzer/SetState' to set the desired analysis state.

    Args:
        None

    Attributes:
        publisher (rclpy.publisher.Publisher): Publisher for publishing analyzed frames.
        subscriber (rclpy.subscription.Subscription): Subscriber for receiving video frames.
        srv (rclpy.service.Service): Service for setting the desired analysis state.
        bridge (CvBridge): Bridge for converting between OpenCV images and ROS2 Image messages.
        img_out (numpy.ndarray): Output image after analysis.
        cv2_img (numpy.ndarray): Input image received from the subscriber.
        active_analysis (AnalysisType): Current active analysis type.
        pose_landmarker (PoseLandmarker): Instance of the PoseLandmarker class for pose landmarking analysis.
        gesture_recognizer (GestureRecognizer): Instance of the GestureRecognizer class for gesture recognition analysis.
    """

    def __init__(self):
        super().__init__('video_analyzer')
        self.publisher = self.create_publisher(Image, 'video_analysis/result', 10)
        self.subscriber = self.create_subscription(Image, 'video/camera', self.callback, 10)
        self.srv = self.create_service(SetState, 'video_analyzer/SetState', self.set_state_callback)
        self.bridge = CvBridge()

        self.img_out = None
        self.cv2_img = None
        self.active_analysis = AnalysisType.GESTURE_RECOGNIZER
        # NOTE: The performance could be improved by not having both analysis running at the same time.
        self.pose_landmarker = PoseLandmarker()
        self.gesture_recognizer = GestureRecognizer()


    def callback(self, msg):
        """
        Callback function for processing incoming video frames and publishing the resulting images to the 'video_analysis/result' topic.

        Args:
            msg (sensor_msgs.msg.Image): Incoming video frame.

        Returns:
            None
        """
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
        """
        Publishes the analyzed frame to the 'video_analysis/result' topic.

        Args:
            None

        Returns:
            None
        """
        # Converts an OpenCV image to a ROS2 Image Message (docs ROS: sensor_msgs/Image.msg)
        msg = self.bridge.cv2_to_imgmsg(self.img_out, 'bgr8')
        # Publishes to the frame topic
        self.publisher.publish(msg)

    def destroy_node(self):
        """
        Cleans up resources and shuts down the node.

        Args:
            None

        Returns:
            None
        """
        self.pose_landmarker.detector.close()
        self.gesture_recognizer.recognizer.close()
        super().destroy_node()
        
    def set_state_callback(self, request, response):
        """
        Callback function for the 'video_analyzer/SetState' service.

        Args:
            request (aida_interfaces.srv.SetState.Request): Service request containing the desired analysis state.
            response (aida_interfaces.srv.SetState.Response): Service response indicating the success or failure of the request.

        Returns:
            aida_interfaces.srv.SetState.Response: Service response indicating the success or failure of the request.
        """
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
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()