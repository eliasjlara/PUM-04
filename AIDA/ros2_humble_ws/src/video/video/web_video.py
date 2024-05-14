import cv2
import rclpy
from rclpy.node import Node
from aida_interfaces.srv import SetState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VideoPublisherNode(Node):
    """
    A ROS2 node that publishes video frames as ROS2 Image messages.

    This node reads frames from a video source and publishes them as ROS2 Image messages on the 'camera' topic.

    Attributes:
        bridge (CvBridge): OpenCV bridge for converting frames to ROS2 Image messages.
        cap (cv2.VideoCapture): Video capture object for reading frames from the video source.
        srv (rclpy.Service): ROS2 service for setting the state of the image publishing.
        publisher_ (rclpy.Publisher): ROS2 publisher for publishing the Image messages.
        timer_period (float): Time period in seconds for the image publishing timer.
        is_active (bool): Flag indicating whether the image publishing is active or not.
        timer (rclpy.Timer): ROS2 timer for triggering the image publishing at regular intervals.
    """

    def __init__(self):
        super().__init__('image_publisher', namespace='video')
        self.bridge = CvBridge()
        source = "https://videos.pexels.com/video-files/3959718/3959718-hd_1366_720_50fps.mp4"
        self.cap = cv2.VideoCapture(source)
        self.srv = self.create_service(SetState, 'SetState', self.set_state_callback)
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.timer_period = 0.03
        self.is_active = True  # Begins as active
        self.timer = self.create_timer(self.timer_period, self.publish_image)

    def publish_image(self):
        """
        Publishes the next frame from the video source as a ROS2 Image message.

        If the image publishing is active, this method reads the next frame from the video source,
        converts it to a ROS2 Image message, and publishes it on the 'camera' topic.
        If the end of the video is reached, the video source is reset to the beginning.

        Returns:
            None
        """
        frame_num = 0
        if self.is_active:
            ret, frame = self.cap.read()
            frame_num += 1
            if ret == True: 
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg.header.frame_id = str(frame_num)
                self.publisher_.publish(msg)
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def set_state_callback(self, request, response):
        """
        Callback function for the 'SetState' service.

        This function is called when a request is made to set the state of the image publishing.
        It updates the 'is_active' flag based on the desired state and sets the response message accordingly.

        Args:
            request (SetState.Request): The request object containing the desired state.
            response (SetState.Response): The response object to be filled with the success status and message.

        Returns:
            SetState.Response: The response object with the updated success status and message.
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
    print("main")
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