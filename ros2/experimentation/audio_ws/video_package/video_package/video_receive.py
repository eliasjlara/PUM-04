import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class VideoReceiveNode(Node):
    def __init__(self):
        super().__init__('video_receive')
        self.subscriber = self.create_subscription(Image, 'video', self.callback, 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        print("received video data")
        print("reshaping video data")
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        print("updating video window") 
        cv2.imshow('video', cv_image) // Creating (or updating if called once before) a window to display the video data
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VideoReceiveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Video Receiver: Keyboard interrupt')

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()