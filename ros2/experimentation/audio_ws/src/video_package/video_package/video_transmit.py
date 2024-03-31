import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

#region MediaPipe Pose Landmarking
import argparse
import sys
import time

import cv2
import mediapipe as mp
import numpy as np

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2
#endregion


#region MediaPipe Pose Landmarking
class PoseLandmarkerNode():
    def __init__(self, cap):
        
        
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        # Global variables to calculate FPS
        self.COUNTER, self.FPS = 0, 0
        self.START_TIME = time.time()
        self.DETECTION_RESULT = None

        # Test
        self.cap = cap
        self.detector = None
        self.output_segmentation_masks = False

        self.setup()

    # Sets up the arguments for the pose landmarker
    def setup(self):
        parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        parser.add_argument(
            '--model',
            help='Name of the pose landmarker model bundle.',
            required=False,
            default='src//models//pose_landmarker_lite.task'
        )
        parser.add_argument(
            '--numPoses',
            help='Max number of poses that can be detected by the landmarker.',
            required=False,
            default=1)
        parser.add_argument(
            '--minPoseDetectionConfidence',
            help='The minimum confidence score for pose detection to be considered '
                'successful.',
            required=False,
            default=0.5)
        parser.add_argument(
            '--minPosePresenceConfidence',
            help='The minimum confidence score of pose presence score in the pose '
                'landmark detection.',
            required=False,
            default=0.5)
        parser.add_argument(
            '--minTrackingConfidence',
            help='The minimum confidence score for the pose tracking to be '
                'considered successful.',
            required=False,
            default=0.5)
        parser.add_argument(
            '--outputSegmentationMasks',
            help='Set this if you would also like to visualize the segmentation '
                'mask.',
            required=False,
            action='store_true')
        # Finding the camera ID can be very reliant on platform-dependent methods.
        # One common approach is to use the fact that camera IDs are usually indexed sequentially by the OS, starting from 0.
        # Here, we use OpenCV and create a VideoCapture object for each potential ID with 'cap = cv2.VideoCapture(i)'.
        # If 'cap' is None or not 'cap.isOpened()', it indicates the camera ID is not available.
        parser.add_argument(
            '--cameraId', help='Id of camera.', required=False, default=0)
        parser.add_argument(
            '--frameWidth',
            help='Width of frame to capture from camera.',
            required=False,
            default=1280)
        parser.add_argument(
            '--frameHeight',
            help='Height of frame to capture from camera.',
            required=False,
            default=960)
        args = parser.parse_args()

        self.setup_camera_feed(args.model, int(args.numPoses), args.minPoseDetectionConfidence,
            args.minPosePresenceConfidence, args.minTrackingConfidence,
            args.outputSegmentationMasks,
            int(args.cameraId), args.frameWidth, args.frameHeight)
    
    def setup_camera_feed(self, model: str, num_poses: int,
        min_pose_detection_confidence: float,
        min_pose_presence_confidence: float, min_tracking_confidence: float,
        output_segmentation_masks: bool,
        camera_id: int, width: int, height: int) -> None:
        """Continuously run inference on images acquired from the camera.

        Args:
        model: Name of the pose landmarker model bundle.
        num_poses: Max number of poses that can be detected by the landmarker.
        min_pose_detection_confidence: The minimum confidence score for pose
            detection to be considered successful.
        min_pose_presence_confidence: The minimum confidence score of pose
            presence score in the pose landmark detection.
        min_tracking_confidence: The minimum confidence score for the pose
            tracking to be considered successful.
        output_segmentation_masks: Choose whether to visualize the segmentation
            mask or not.
        camera_id: The camera id to be passed to OpenCV.
        width: The width of the frame captured from the camera.
        height: The height of the frame captured from the camera.
        """

        self.output_segmentation_masks = output_segmentation_masks

        # Start capturing video input from the camera
        #self.cap = cv2.VideoCapture(camera_id) Moved
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Visualization parameters
        fps_avg_frame_count = 10

        def save_result(result: vision.PoseLandmarkerResult,
                        unused_output_image: mp.Image, timestamp_ms: int):
            #global FPS, COUNTER, START_TIME, DETECTION_RESULT

            # Calculate the FPS
            if self.COUNTER % fps_avg_frame_count == 0:
                self.FPS = fps_avg_frame_count / (time.time() - self.START_TIME)
                self.START_TIME = time.time()

            self.DETECTION_RESULT = result
            self.COUNTER += 1

        # Initialize the pose landmarker model
        base_options = python.BaseOptions(model_asset_path=model)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            num_poses=num_poses,
            min_pose_detection_confidence=min_pose_detection_confidence,
            min_pose_presence_confidence=min_pose_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
            output_segmentation_masks=output_segmentation_masks,
            result_callback=save_result)
        self.detector = vision.PoseLandmarker.create_from_options(options)
    
    def apply_pose_landmarking(self) -> np.ndarray:
        success, image = self.cap.read()
        if not success:
            sys.exit(
                'ERROR: Unable to read from webcam. Please verify your webcam settings.'
            )

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # Run pose landmarker using the model.
        self.detector.detect_async(mp_image, time.time_ns() // 1_000_000)

        # Visualization parameters
        row_size = 50  # pixels
        left_margin = 24  # pixels
        text_color = (0, 0, 0)  # black
        font_size = 1
        font_thickness = 1
        overlay_alpha = 0.5
        mask_color = (100, 100, 0)  # cyan

        # Show the FPS
        fps_text = 'FPS = {:.1f}'.format(self.FPS)
        text_location = (left_margin, row_size)
        current_frame = image
        cv2.putText(current_frame, fps_text, text_location,
                    cv2.FONT_HERSHEY_DUPLEX,
                    font_size, text_color, font_thickness, cv2.LINE_AA)

        if self.DETECTION_RESULT:
            # Draw landmarks.
            for pose_landmarks in self.DETECTION_RESULT.pose_landmarks:
                # Draw the pose landmarks.
                pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                pose_landmarks_proto.landmark.extend([
                    landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y,
                                                    z=landmark.z) for landmark
                    in pose_landmarks
                ])
                self.mp_drawing.draw_landmarks(
                    current_frame,
                    pose_landmarks_proto,
                    self.mp_pose.POSE_CONNECTIONS,
                    self.mp_drawing_styles.get_default_pose_landmarks_style())

        if (self.output_segmentation_masks and self.DETECTION_RESULT):
            if self.DETECTION_RESULT.segmentation_masks is not None:
                segmentation_mask = self.DETECTION_RESULT.segmentation_masks[0].numpy_view()
                mask_image = np.zeros(image.shape, dtype=np.uint8)
                mask_image[:] = mask_color
                condition = np.stack((segmentation_mask,) * 3, axis=-1) > 0.1
                visualized_mask = np.where(condition, mask_image, current_frame)
                current_frame = cv2.addWeighted(current_frame, overlay_alpha,
                                                visualized_mask, overlay_alpha,
                                                0)
        
        return current_frame
              

#endregion


class VideoTransmitNode(Node):
    def __init__(self):
        super().__init__('video_transmit')
        self.publisher = self.create_publisher(Image, 'video', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(
            0.1, self.publish_video)  # Adjust the rate as needed
        # Replace with your video file or stream URL
        #self.cap = cv2.VideoCapture('http://195.196.36.242/mjpg/video.mjpg')
        #self.cap = cv2.VideoCapture('https://media.mammothresorts.com/mmsa/mammoth/cams/Village_Gondola_1280x720.jpg')
        #self.cap = cv2.VideoCapture('https://rr5---sn-5hneknek.googlevideo.com/videoplayback?expire=1711896499&ei=UyMJZvL0J924v_IPpqqe4Ak&ip=155.4.149.196&id=o-AJkOJvKJpZgGu2GUT914-YPCpthOnCYXZ5zdjo5HcpU2&itag=22&source=youtube&requiressl=yes&xpc=EgVo2aDSNQ%3D%3D&spc=UWF9f0qJHXLl2Ln2_HUDrYJqtysKXLOIC2DiVWyEYHsupKA&vprv=1&svpuc=1&mime=video%2Fmp4&ns=K-O66OL5NmmrkK4PCX2ZcroQ&rqh=1&cnr=14&ratebypass=yes&dur=110.341&lmt=1587657090139165&fexp=51141541&c=WEB&sefc=1&txp=6316222&n=LYcmuM4GWabtsdlw&sparams=expire%2Cei%2Cip%2Cid%2Citag%2Csource%2Crequiressl%2Cxpc%2Cspc%2Cvprv%2Csvpuc%2Cmime%2Cns%2Crqh%2Ccnr%2Cratebypass%2Cdur%2Clmt&sig=AJfQdSswRQIhAMzm0uYpfJT7tA-IFSjRGKNfqVHJjrgQGxByMzzEuJhoAiAVWdtjJnPFAEyMqmqAWivs3EgpfLXNTTos76rBtpe0xg%3D%3D&cm2rm=sn-c5ioiv45c-hhme7s,sn-c5ioiv45c-5gol7e,sn-5golr7e&req_id=54a3154540e1a3ee&redirect_counter=3&cms_redirect=yes&cmsv=e&mh=gx&mm=34&mn=sn-5hneknek&ms=ltu&mt=1711874663&mv=m&mvi=5&pl=20&lsparams=mh,mm,mn,ms,mv,mvi,pl&lsig=ALClDIEwRQIgAWilUnYEyMVImp3k15p3mVFWroDHizwRVY3OmYq9rbUCIQDCS2bRGCQNLzdC84dMQ0bkBWmBV40mXtxyvkvqEb_tvQ%3D%3D')
        self.cap = cv2.VideoCapture("1person.mp4")

        # Initialize the PoseLandmarkerNode with the video capture
        self.pose_landmarker = PoseLandmarkerNode(self.cap)

    def publish_video(self):
        #region Test remove
        
        ret, frame = self.cap.read()
        if ret:
            # Converts an OpenCV image to a ROS2 Image Message (docs ROS: sensor_msgs/Image.msg)
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            # Publishes to the 'video' topic
            self.publisher.publish(msg)
        
        #endregion
        frame = self.pose_landmarker.apply_pose_landmarking()
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
    node.pose_landmarker.detector.close()
    node.cap.release()
    cv2.destroyAllWindows()
    # Node cleanup
    node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()
