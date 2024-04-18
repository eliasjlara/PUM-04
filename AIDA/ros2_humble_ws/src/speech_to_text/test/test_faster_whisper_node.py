# from speech_to_text.faster_whisper_node import STTNode
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from audio_data.msg import AudioData
# import pytest


# @pytest.fixture
# def node():
#     rclpy.init()
#     yield STTNode()
#     rclpy.shutdown()

# def test_node_creation(node):
#     assert node is not None

# def test_message_to_numpy_array(node):
#     """
#     Tests the message_to_numpy_array method in the FasterWhisperLogic class.
#     Tests with a numpy array with 4 elements.
#     """
 

#     data = np.ndarray(shape=(4,), dtype=np.float32)
#     data[0] = 1.32
#     data[1] = 2.44
#     data[2] = 3.14
#     data[3] = 4.1276
#     msg = AudioData()
#     msg.data = data.tobytes()
#     msg.samples = len(data)    
    
#     # Call the message_to_numpy_array method
#     result = node._message_to_numpy_array(msg)

#     assert np.array_equal(result, data)

# def test_message_to_numpy_array_empty(node):
#     """
#     Tests the message_to_numpy_array method in the FasterWhisperLogic class.
#     Tests with an empty numpy array.
#     """
#     data = np.empty(dtype=np.float32, shape=(0,))
    
#     #data = np.ndarray(dtype=np.float32, shape=(0))
    
#     msg = AudioData()
#     msg.data = data.tobytes()
#     msg.samples = len(data)
#     # Call the message_to_numpy_array method
#     with pytest.raises(ValueError) as exec_info:
#         result = node._message_to_numpy_array(msg)
#     assert exec_info.type == ValueError
