import cv2
import rclpy
from rclpy.node import Node
from aida_interfaces.srv import SetState
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class VideoPublisherNode(Node):
    """
    A ROS2 node that publishes video frames from a camera.

    This node captures frames from a camera and publishes them as ROS2 Image messages.

    Attributes:
        bridge (CvBridge): OpenCV bridge for converting images.
        cap (cv2.VideoCapture): Video capture object for accessing the camera.
        srv (rclpy.Service): ROS2 service for setting the state of the image publishing.
        publisher_ (rclpy.Publisher): ROS2 publisher for publishing the camera images.
        timer_period (float): Time period for the image publishing timer.
        is_active (bool): Flag indicating whether the image publishing is active or not.
        timer (rclpy.Timer): ROS2 timer for triggering the image publishing at regular intervals.
    """

    def __init__(self):
        super().__init__('image_publisher', namespace='video')
        self.bridge = CvBridge()
        width = 640
        height = 480
        self.cap = cv2.VideoCapture(0)
        # Set capture property for frame width
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        # Set capture property for frame height
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.srv = self.create_service(SetState, 'SetState', self.set_state_callback)
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.timer_period = 0.05 # 0.05 => 20 Hz
        self.is_active = True  # Begins as active
        self.timer = self.create_timer(self.timer_period, self.publish_image)

    def publish_image(self):
        """
        Publishes the camera image as a ROS2 Image message.

        This method reads a frame from the camera, converts it to a ROS2 Image message,
        and publishes it using the ROS2 publisher.
        """
        frame_num = 0
        if self.is_active:
            ret, frame = self.cap.read()
            frame_num += 1
            if ret == True:
                # self.bridge.cv2_to_compressed_imgmsg(frame)
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg.header.frame_id = str(frame_num)
                self.publisher_.publish(msg)

    def set_state_callback(self, request, response):
        """
        Callback function for the SetState service.

        This method is called when the SetState service is invoked.
        It sets the state of the image publishing based on the desired state provided in the request.

        Args:
            request (SetState.Request): The request object containing the desired state.
            response (SetState.Response): The response object to be filled with the result.

        Returns:
            SetState.Response: The response object with the success status and message.
        """
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