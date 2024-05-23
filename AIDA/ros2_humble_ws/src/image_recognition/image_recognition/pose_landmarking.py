import cv2
import threading
import numpy as np
import time

import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2


class PoseLandmarkerWrapper:
    """
    Object that performs pose landmarking on the captured images.

    Uses the MediaPipe pose landmarking model to detect human body landmarks in the images.
    https://developers.google.com/mediapipe/solutions/vision/pose_landmarker/python
    """

    def __init__(self, load: bool = False) -> None:

        self.label_text_color = (255, 255, 255)  # white
        self.label_font_size = 1
        self.label_thickness = 2

        self.model = "..//src//image_recognition//models//pose_landmarker_lite.task"
        self.result = None
        self.result_lock = threading.Lock()

        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        BaseOptions = mp.tasks.BaseOptions
        PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        self.options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.model),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.save_result,
        )

        if load:
            self.load_model()

    def load_model(self) -> None:
        """
        Loads the pose landmarking model.

        Args:
            None

        Returns:
            None
        """
        PoseLandmarker = mp.tasks.vision.PoseLandmarker
        self.landmarker = PoseLandmarker.create_from_options(self.options)

    def unload_model(self) -> None:
        """
        Unloads the pose landmarking model.

        Args:
            None

        Returns:
            None
        """
        self.landmarker = None

    def save_result(
        self,
        result: vision.PoseLandmarkerResult,
        output_image: mp.Image,
        timestamp_ms: int,
    ) -> None:
        """
        Save the pose landmarking result.

        Used as a callback function for the pose landmarking model.

        Args:
            result : PoseLandmarkerResult : The result of the pose landmarking.
            unused_output_image : mp.Image : The output image.
            timestamp_ms : int : The timestamp of the result.

        Returns:
            None
        """
        with self.result_lock:
            self.result = result

    def apply_result(self, cv2_img) -> np.ndarray:
        """
        Returns the image with the pose landmarking results.

        Args:
            None

        Returns:
            np.ndarray : The image with the pose landmarking results.
        """
        if self.result:
            with self.result_lock:
                for pose_landmarks in self.result.pose_landmarks:
                    self.draw_landmarks(cv2_img, pose_landmarks)
        return cv2_img

    def apply_pose_landmarking(self, cv2_img: cv2.VideoCapture) -> np.ndarray:
        """
        Applies pose landmarking to the input image.

        Args:
            cv2_img : cv2.VideoCapture : The input image to apply pose landmarking to.

        Returns:
            cv2.VideoCapture : The input image with the pose landmarking applied.
        """
        np_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=np_img)
        self.landmarker.detect_async(mp_img, time.time_ns() // 1_000_000)

    def draw_landmarks(self, cv2_img, landmarks):
        """
        Draws the pose landmarks on the frame using MediaPipe drawing functions.

        Args:
            current_frame: The OpenCV image frame.

        Returns:
            The image frame with the pose landmarks drawn on it.
        """
        pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        pose_landmarks_proto.landmark.extend([
        landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in landmarks
        ])
        self.mp_drawing.draw_landmarks(
            cv2_img,
            pose_landmarks_proto,
            self.mp_pose.POSE_CONNECTIONS,
            self.mp_drawing_styles.get_default_pose_landmarks_style())
        return cv2_img