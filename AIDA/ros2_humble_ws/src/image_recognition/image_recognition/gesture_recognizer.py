import argparse
import sys
import time

import cv2
import mediapipe as mp
import numpy as np

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2


class GestureRecognizer():
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Global variables to calculate FPS
        self.COUNTER, self.FPS = 0, 0
        self.START_TIME = time.time()
        self.model = "..//src//image_recognition//models//gesture_recognizer.task"
        self.num_hands = 2
        self.min_hand_detection_confidence = 0.5
        self.min_hand_presence_confidence = 0.5
        self.min_tracking_confidence = 0.5
        self.camera_id = 0
        self.desired_width = 640
        self.desired_height = 480

        # Test
        self.recognizer = None
        self.recognition_result_list = []

        self.setup()

    # Sets up the arguments for the gesture recognizer
    def setup(self):
        self.set_up_camera_feed(self.model, self.num_hands, self.min_hand_detection_confidence,
                                self.min_hand_presence_confidence, self.min_tracking_confidence, self.camera_id,
                                self.desired_width, self.desired_height)

    def set_up_camera_feed(self, model: str, num_hands: int,
        min_hand_detection_confidence: float,
        min_hand_presence_confidence: float, min_tracking_confidence: float,
        camera_id: int, width: int, height: int) -> None:
        """Continuously run inference on images acquired from the camera.

        Args:
            model: Name of the gesture recognition model bundle.
            num_hands: Max number of hands can be detected by the recognizer.
            min_hand_detection_confidence: The minimum confidence score for hand
                detection to be considered successful.
            min_hand_presence_confidence: The minimum confidence score of hand
                presence score in the hand landmark detection.
            min_tracking_confidence: The minimum confidence score for the hand
                tracking to be considered successful.
            camera_id: The camera id to be passed to OpenCV.
            width: The width of the frame captured from the camera.
            height: The height of the frame captured from the camera.
        """

        self.desired_width = width
        self.desired_height = height
        # Visualization parameters
        fps_avg_frame_count = 10

        recognition_frame = None
        #recognition_result_list = []
        self.recognition_result_list.clear()

        def save_result(result: vision.GestureRecognizerResult,
                        unused_output_image: mp.Image, timestamp_ms: int):
                        # Calculate the FPS
            if self.COUNTER % fps_avg_frame_count == 0:
                self.FPS = fps_avg_frame_count / (time.time() - self.START_TIME)
                self.START_TIME = time.time()

            self.recognition_result_list.append(result)
            self.COUNTER += 1

        # Initialize the gesture recognizer model
        base_options = python.BaseOptions(model_asset_path=model)
        options = vision.GestureRecognizerOptions(base_options=base_options,
                                                running_mode=vision.RunningMode.LIVE_STREAM,
                                                num_hands=num_hands,
                                                min_hand_detection_confidence=min_hand_detection_confidence,
                                                min_hand_presence_confidence=min_hand_presence_confidence,
                                                min_tracking_confidence=min_tracking_confidence,
                                                result_callback=save_result)
        self.recognizer = vision.GestureRecognizer.create_from_options(options)

    def _crop_image(self, image):
        height, width = image.shape[:2]  # Get original height and width
        # Calculate how much to crop from the sides and top/bottom
        x_start = (width - self.desired_width) // 2
        y_start = (height - self.desired_height) // 2
        x_end = x_start + self.desired_width
        y_end = y_start + self.desired_height

        # Perform the cropping using array slicing
        cropped_img = image[y_start:y_end, x_start:x_end]

        return cropped_img
    
    def apply_gesture_detection(self, image) -> np.ndarray:
        
        image = self._crop_image(image)        

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # Run gesture recognizer using the model.
        self.recognizer.recognize_async(mp_image, time.time_ns() // 1_000_000)

        #Visualization parameters
        row_size = 50  # pixels
        left_margin = 24  # pixels
        text_color = (0, 0, 0)  # black
        font_size = 1
        font_thickness = 1

        # Label box parameters
        label_text_color = (255, 255, 255)  # white
        label_font_size = 1
        label_thickness = 2

        # Show the FPS
        fps_text = 'FPS = {:.1f}'.format(self.FPS)
        text_location = (left_margin, row_size)
        current_frame = image
        cv2.putText(current_frame, fps_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                    font_size, text_color, font_thickness, cv2.LINE_AA)
        
        if self.recognition_result_list:
            # Draw landmarks and write the text for each hand.
            for hand_index, hand_landmarks in enumerate(
                self.recognition_result_list[0].hand_landmarks):
                 # Calculate the bounding box of the hand
                x_min = min([landmark.x for landmark in hand_landmarks])
                y_min = min([landmark.y for landmark in hand_landmarks])
                y_max = max([landmark.y for landmark in hand_landmarks])

                # Convert normalized coordinates to pixel values
                frame_height, frame_width = current_frame.shape[:2]
                x_min_px = int(x_min * frame_width)
                y_min_px = int(y_min * frame_height)
                y_max_px = int(y_max * frame_height)

                # Get gesture classification results
                if self.recognition_result_list[0].gestures:
                    gesture = self.recognition_result_list[0].gestures[hand_index]
                    category_name = gesture[0].category_name
                    score = round(gesture[0].score, 2)
                    result_text = f'{category_name} ({score})'

                    # Compute text size
                    text_size = \
                    cv2.getTextSize(result_text, cv2.FONT_HERSHEY_DUPLEX, label_font_size,
                                    label_thickness)[0]
                    text_width, text_height = text_size

                    # Calculate text position (above the hand)
                    text_x = x_min_px
                    text_y = y_min_px - 10  # Adjust this value as needed

                    # Make sure the text is within the frame boundaries
                    if text_y < 0:
                        text_y = y_max_px + text_height

                    # Draw the text
                    cv2.putText(current_frame, result_text, (text_x, text_y),
                                cv2.FONT_HERSHEY_DUPLEX, label_font_size,
                                label_text_color, label_thickness, cv2.LINE_AA)
                # Draw hand landmarks on the frame
                hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                hand_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y,
                                                z=landmark.z) for landmark in
                hand_landmarks
                ])
                self.mp_drawing.draw_landmarks(
                current_frame,
                hand_landmarks_proto,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                self.mp_drawing_styles.get_default_hand_connections_style())

            #recognition_frame = current_frame
            self.recognition_result_list.clear()
        recognition_frame = current_frame
        return recognition_frame