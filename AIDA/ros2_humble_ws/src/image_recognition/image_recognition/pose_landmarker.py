import time
import cv2
import mediapipe as mp
import numpy as np

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2


class PoseLandmarker():
    def __init__(self):
        """
        Initializes the PoseLandmarker class.
        """
        
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        # Global variables to calculate FPS
        self.COUNTER, self.FPS = 0, 0
        self.START_TIME = time.time()
        self.DETECTION_RESULT = None
        self.model = "..//src//image_recognition//models//pose_landmarker_lite.task"
        self.num_poses = 1
        self.min_pose_detection_confidence = 0.5
        self.min_pose_presence_confidence = 0.5
        self.min_tracking_confidence = 0.5
        self.camera_id = 0
        self.desired_width = 1280
        self.desired_height = 960

        self.detector = None
        self.output_segmentation_masks = False

        self.setup_camera_feed(self.model, self.num_poses, self.min_pose_detection_confidence,
                               self.min_pose_presence_confidence, self.min_tracking_confidence,
                               self.output_segmentation_masks, self.desired_width, self.desired_height)
    
    def setup_camera_feed(self, model: str, num_poses: int,
        min_pose_detection_confidence: float,
        min_pose_presence_confidence: float, min_tracking_confidence: float,
        output_segmentation_masks: bool,
        width: int, height: int) -> None:
        """
        Continuously run inference on images.

        Args:
        - model: Name of the pose landmarker model bundle.
        - num_poses: Max number of poses that can be detected by the landmarker.
        - min_pose_detection_confidence: The minimum confidence score for pose
            detection to be considered successful.
        - min_pose_presence_confidence: The minimum confidence score of pose
            presence score in the pose landmark detection.
        - min_tracking_confidence: The minimum confidence score for the pose
            tracking to be considered successful.
        - output_segmentation_masks: Choose whether to visualize the segmentation
            mask or not.
        - width: The width of the frame captured from the camera.
        - height: The height of the frame captured from the camera.
        """
        self.output_segmentation_masks = output_segmentation_masks

        self.desired_width = width
        self.desired_height = height
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
    
    def _crop_image(self, image):
        """
        Crop the image to the desired width and height.

        Args:
        - image: The input image.

        Returns:
        - cropped_img: The cropped image.
        """
        height, width = image.shape[:2]  # Get original height and width
        dim_scales = [self.desired_height / height, self.desired_width / width]
        image = image.resize(width*max(dim_scales), height*max(dim_scales))
        # Calculate how much to crop from the sides and top/bottom
        x_start = (width - self.desired_width) // 2
        y_start = (height - self.desired_height) // 2
        x_end = x_start + self.desired_width
        y_end = y_start + self.desired_height
        print(f"Image shape: {image.shape}")

        # Perform the cropping using array slicing
        cropped_img = image[y_start:y_end, x_start:x_end]

        return cropped_img
    

    def apply_pose_landmarking(self, image) -> np.ndarray:
        """
        Apply pose landmarking to the input image.

        Args:
        - image: The input image.

        Returns:
        - current_frame: The image with pose landmarks and segmentation masks (if enabled) drawn on it.
        """
        image = self._crop_image(image)

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
        
              
