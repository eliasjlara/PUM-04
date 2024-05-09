import queue
import threading
import cv2
import time

import rclpy
from rclpy.node import Node
from aida_interfaces.srv import SetState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VideoPublisherNode(Node):
    """
    A ROS2 node for transmitting images from camera.

    This node initializes a publisher to capture images from the camera and publish them to a ROS2 topic.
    It provides methods to start and stop the capture and publisher workers.
    """
    def __init__(self):
            super().__init__('image_publisher', namespace='video')
            self.bridge = CvBridge()
            self.cap = cv2.VideoCapture(0)
            self.srv = self.create_service(SetState, 'SetState', self.set_state_callback)
            self.publisher_ = self.create_publisher(Image, 'camera', 10)
            self.timer_period = 0.03
            self.is_active = True  # Begins as active
            self.timer = self.create_timer(self.timer_period, self.publish_image)

    def publish_image(self):
        frame_num = 0
        if self.is_active:
            ret, frame = self.cap.read()
            frame_num += 1
            if ret == True:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg.header.frame_id = str(frame_num)
                self.publisher_.publish(msg)

    def set_state_callback(self, request, response):
        if request.desired_state == "active":
            self.is_active = True
            response.success = True
            response.message = "Image publishing activated"
        elif request.desired_state == "idle":
            self.is_active = False
            response.success = True
            response.message = "Image publishing deactivated"
        else:
            response.success = False
            response.message = "Invalid desired state"
        return response
    
def main(args=None):
    rclpy.init(args=args)

    video_node = VideoPublisherNode()

    try:
        rclpy.spin(video_node)
    except KeyboardInterrupt:
        video_node.get_logger().info('Keyboard interrupt')

    video_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()