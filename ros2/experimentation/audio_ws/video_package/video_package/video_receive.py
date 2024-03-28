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
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('video', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VideoReceiveNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()