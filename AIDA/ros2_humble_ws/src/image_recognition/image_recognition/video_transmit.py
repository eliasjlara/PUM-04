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
        self.subscriber = self.create_subscription(Image, 'video', self.callback, 10)
        self.bridge = CvBridge()

        self.timer = self.create_timer(
            0.1, self.publish_video)  # Adjust the rate as needed
        # Replace with your video file or stream URL
        self.cap = cv2.VideoCapture('https://videos.pexels.com/video-files/854370/854370-sd_640_360_30fps.mp4')
        #self.cap = cv2.VideoCapture('http://195.196.36.242/mjpg/video.mjpg')
        #self.cap = cv2.VideoCapture('https://media.mammothresorts.com/mmsa/mammoth/cams/Village_Gondola_1280x720.jpg')
        #self.cap = cv2.VideoCapture('https://rr5---sn-5hneknek.googlevideo.com/videoplayback?expire=1711896499&ei=UyMJZvL0J924v_IPpqqe4Ak&ip=155.4.149.196&id=o-AJkOJvKJpZgGu2GUT914-YPCpthOnCYXZ5zdjo5HcpU2&itag=22&source=youtube&requiressl=yes&xpc=EgVo2aDSNQ%3D%3D&spc=UWF9f0qJHXLl2Ln2_HUDrYJqtysKXLOIC2DiVWyEYHsupKA&vprv=1&svpuc=1&mime=video%2Fmp4&ns=K-O66OL5NmmrkK4PCX2ZcroQ&rqh=1&cnr=14&ratebypass=yes&dur=110.341&lmt=1587657090139165&fexp=51141541&c=WEB&sefc=1&txp=6316222&n=LYcmuM4GWabtsdlw&sparams=expire%2Cei%2Cip%2Cid%2Citag%2Csource%2Crequiressl%2Cxpc%2Cspc%2Cvprv%2Csvpuc%2Cmime%2Cns%2Crqh%2Ccnr%2Cratebypass%2Cdur%2Clmt&sig=AJfQdSswRQIhAMzm0uYpfJT7tA-IFSjRGKNfqVHJjrgQGxByMzzEuJhoAiAVWdtjJnPFAEyMqmqAWivs3EgpfLXNTTos76rBtpe0xg%3D%3D&cm2rm=sn-c5ioiv45c-hhme7s,sn-c5ioiv45c-5gol7e,sn-5golr7e&req_id=54a3154540e1a3ee&redirect_counter=3&cms_redirect=yes&cmsv=e&mh=gx&mm=34&mn=sn-5hneknek&ms=ltu&mt=1711874663&mv=m&mvi=5&pl=20&lsparams=mh,mm,mn,ms,mv,mvi,pl&lsig=ALClDIEwRQIgAWilUnYEyMVImp3k15p3mVFWroDHizwRVY3OmYq9rbUCIQDCS2bRGCQNLzdC84dMQ0bkBWmBV40mXtxyvkvqEb_tvQ%3D%3D')
        #self.cap = cv2.VideoCapture("1person.mp4")
        #self.cap = cv2.VideoCapture("https://rr6---sn-c5ioiv45c-hhme.googlevideo.com/videoplayback?expire=1712072149&ei=ddELZtOVB7mPv_IPvKGD6As&ip=155.4.149.196&id=o-AMlBBnvBj8JhJ3BoXsuJbVzK-P2LUMsVkjrHTejw7gVh&itag=18&source=youtube&requiressl=yes&xpc=EgVo2aDSNQ%3D%3D&mh=ty&mm=31%2C29&mn=sn-c5ioiv45c-hhme%2Csn-c5ioiv45c-5goe&ms=au%2Crdu&mv=m&mvi=6&pl=20&initcwndbps=3033750&spc=UWF9fy_N0DhZ5zwflagqPelj-qXroPHRa8FyNLq5KbjFDgQ&vprv=1&svpuc=1&mime=video%2Fmp4&ns=psJAQiT_19MKdHE0cheaDEYQ&cnr=14&ratebypass=yes&dur=264.312&lmt=1658394672193702&mt=1712050153&fvip=7&fexp=51141541&c=WEB&sefc=1&txp=5318224&n=djmBWPBzNAIJaYx1&sparams=expire%2Cei%2Cip%2Cid%2Citag%2Csource%2Crequiressl%2Cxpc%2Cspc%2Cvprv%2Csvpuc%2Cmime%2Cns%2Ccnr%2Cratebypass%2Cdur%2Clmt&sig=AJfQdSswRQIhAODBIEQh133LkwHrjkuI6ZTq9UUyGkwL-XX4hPdsI_iPAiBn7jkvTxO8gn8OvxKEaKnS10oHSDQNss9j1C0hzo2TRw%3D%3D&lsparams=mh%2Cmm%2Cmn%2Cms%2Cmv%2Cmvi%2Cpl%2Cinitcwndbps&lsig=ALClDIEwRQIhAPPqDSYvbJrF8tzXqHB9iXH21xIdOQZXcEvLRcp0lD_jAiBUd4Zbrr-sW8sLjw4EHaydOd-9Ste5_fdVtQT35jyIaQ%3D%3D")

        # Initialize the PoseLandmarkerNode with the video capture
        self.pose_landmarker = PoseLandmarker(self.cap) # XXX
        #self.gesture_recognizer = GestureRecognizer(self.cap)

    def callback(self, msg):
        # received video data
        # converting video data

        self.cap = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.pose_landmarker = PoseLandmarker(self.cap)

        #imS = cv2.resize(cv_image, (960, 540)) 
        # updating video window 
        #cv2.imshow('video', imS) # Creating (or updating if called once before) a window to display the video data
        #cv2.waitKey(1)

        self.publish_video()

    def publish_video(self):
        #region Test remove
        
        ret, frame = self.cap.read()
        if ret:
            # Converts an OpenCV image to a ROS2 Image Message (docs ROS: sensor_msgs/Image.msg)
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            # Publishes to the 'video' topic
            self.publisher.publish(msg)
        
        #endregion
        #frame = self.gesture_recognizer.apply_gesture_detection()
        frame = self.pose_landmarker.apply_pose_landmarking() # XXX
        #success, frame = self.cap.read()


        #region Test for local video analysis
        """
        imS = cv2.resize(frame, (960, 540)) 
        # updating video window 
        cv2.imshow('video', imS) # Creating (or updating if called once before) a window to display the video data
        cv2.waitKey(1)
        """
        #end region


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

    # Call cleanup functions when Ctrl+C is pressed
    node.pose_landmarker.detector.close() # XXX
    #node.cap.release()
    #cv2.destroyAllWindows()
    #node.gesture_recognizer.recognizer.close()
    #node.cap.release()
    cv2.destroyAllWindows()
    # Node cleanup
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
