import cv2
import threading
import numpy as np
import time

import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2

class GestureRecognizerWrapper:
    """
    Object that performs gesture recognition on the captured images.

    Uses the MediaPipe gesture recognition model to detect hands in the images and classify the gestures.
    https://developers.google.com/mediapipe/solutions/vision/gesture_recognizer/python
    """

    def __init__(self, load: bool = False) -> None:

        self.label_text_color = (255, 255, 255)  # white
        self.label_font_size = 1
        self.label_thickness = 2

        self.gesture_options = {
            "num_hands": 2,
        }

        self.model = "..//src//image_recognition//models//gesture_recognizer.task"
        self.result = None
        self.result_lock = threading.Lock()

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        BaseOptions = mp.tasks.BaseOptions
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        self.options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=self.model),
            running_mode=VisionRunningMode.LIVE_STREAM,
            num_hands=self.gesture_options["num_hands"],
            result_callback=self.save_result,
        )

        if load:
            self.load_model()

    def load_model(self) -> None:
        """
        Loads the gesture recognition model.

        Args:
            None

        Returns:
            None
        """
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        self.recognizer = GestureRecognizer.create_from_options(self.options)

    def unload_model(self) -> None:
        """
        Unloads the gesture recognition model.

        Args:
            None

        Returns:
            None
        """
        self.recognizer = None
    
    def save_result(
        self,
        result: vision.GestureRecognizerResult,
        output_image: mp.Image,
        timestamp_ms: int,
    ) -> None :
        """
        Save the gesture recognition result.

        Used as a callback function for the gesture recognition model.

        Args:
            result : GestureRecognizerResult : The result of the gesture recognition.
            unused_output_image : mp.Image : The output image.
            timestamp_ms : int : The timestamp of the result.

        Returns:
            None
        """
        with self.result_lock:
            self.result = result

    def apply_result(self, cv2_img) -> np.ndarray:
        """
        Returns the image with the gesture recognition results.

        Args:
            None

        Returns:
            np.ndarray : The image with the gesture recognition results.
        """
        if self.result:
            with self.result_lock:
                result = self.result
                # image = self.draw_gesture_results(self.result)
                for hand_index, hand_landmarks in enumerate(self.result.hand_landmarks):
                    self.draw_landmarks(cv2_img, hand_landmarks)
                    self.draw_gesture_text(cv2_img, hand_landmarks, self.result.gestures[hand_index])
        return cv2_img

    def apply_gesture_detection(self, cv2_img: cv2.VideoCapture) -> np.ndarray:
        """
        Applies gesture recognition to the input image.

        Args:
            cv2_img : cv2.VideoCapture : The input image to apply gesture recognition to.

        Returns:
            cv2.VideoCapture : The input image with the gesture recognition applied.
        """
        np_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=np_img)
        self.recognizer.recognize_async(mp_img, time.time_ns() // 1_000_000)

    def get_text_size(self, text, font_size, font_thickness):
        """
        Calculates the size of the text using OpenCV's getTextSize function.

        Args:
            text: The text string to measure.
            font_size: The font size for the text.
            font_thickness: The thickness of the font.

        Returns:
            A tuple containing the width and height of the text.
        """
        return cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, font_size, font_thickness)[0]


    def draw_landmarks(self, cv2_img, hand_landmarks) -> None:
        """
        Draws the hand landmarks on the frame using MediaPipe drawing functions.

        Args:
            current_frame: The OpenCV image frame.
            hand_landmarks: A list of landmark objects representing the hand.

        Returns:
            The image frame with the hand landmarks drawn on it.
        """
        hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        hand_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
        ])
        self.mp_drawing.draw_landmarks(
            cv2_img,
            hand_landmarks_proto,
            self.mp_hands.HAND_CONNECTIONS,
            self.mp_drawing_styles.get_default_hand_landmarks_style(),
            self.mp_drawing_styles.get_default_hand_connections_style())

    def draw_gesture_text(self, cv2_img, hand_landmarks, gesture) -> None:
        """
        Calculates the bounding box, text position, and draws the gesture classification text.

        Args:
            current_frame: The OpenCV image frame.
            hand_landmarks: A list of landmark objects representing the hand.
            gesture: A gesture object containing classification results (assuming single result).

        Returns:
            The image frame with the gesture text drawn on it.
        """
        frame_height, frame_width = cv2_img.shape[:2]

            # Calculate the bounding box of the hand
        x_min = min([landmark.x for landmark in hand_landmarks])
        y_min = min([landmark.y for landmark in hand_landmarks])
        y_max = max([landmark.y for landmark in hand_landmarks])

        # Convert normalized coordinates to pixel values
        x_min_px = int(x_min * frame_width)
        y_min_px = int(y_min * frame_height)
        y_max_px = int(y_max * frame_height)

        # Prepare gesture text (assuming single gesture in `gesture`)
        category_name = gesture[0].category_name
        score = round(gesture[0].score, 2)
        result_text = f'{category_name} ({score})'

        # Calculate text size and position
        text_width, text_height = self.get_text_size(result_text, self.label_font_size, self.label_thickness)
        text_x = x_min_px
        text_y = y_min_px - 10  # Adjust this value as needed

        # Ensure text stays within frame boundaries
        if text_y < 0:
            text_y = y_max_px + text_height

        # Draw the text
        cv2.putText(cv2_img, result_text, (text_x, text_y),
                    cv2.FONT_HERSHEY_DUPLEX, self.label_font_size,
                    self.label_text_color, self.label_thickness, cv2.LINE_AA)

