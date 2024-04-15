# from speech_to_text.faster_whisper_node import STTNode
# import numpy as np
# import rclpy
# from rclpy.node import Node
# import pytest

# def test_msg_to_np():
#     rclpy.init()
    
#     stt = STTNode()
#     #data = np.ndarray([1.32, 2.44, 3.14, 4.1276], dtype=np.float32, shape=(4,))
#     data = np.ndarray(shape=(4,), dtype=np.float32)
#     data[0] = 1.32
#     data[1] = 2.44
#     data[2] = 3.14
#     data[3] = 4.1276

#     msg = AudioData()

#     msg.data = data.tobytes()

#     assert np.array_equal(stt._msg_to_nparray(msg), data)

#     stt.destroy_node()
#     rclpy.shutdown()

# def test_msg_to_np_empty():
#     rclpy.init()
#     stt = STTNode()
#     data = np.ndarray(dtype=np.float32)
#     msg = AudioData()
#     msg.data = data.tobytes()
#     assert np.array_equal(stt._msg_to_nparray(msg), data)

#     stt.destroy_node()
#     rclpy.shutdown()


# def test_translate():
#     rclpy.init()
#     stt = STTNode()
#     stt.destroy_node()
#     rclpy.shutdown()

# def test_message_to_numpy_array():
#     """
#     Tests the message_to_numpy_array method in the FasterWhisperLogic class.
#     Tests with a numpy array with 4 elements.
#     """
#     # Create a FasterWhisperLogic object
    
#     rclpy.init()
#     stt = STTNode()   

#     data = np.ndarray(shape=(4,), dtype=np.float32)
#     data[0] = 1.32
#     data[1] = 2.44
#     data[2] = 3.14
#     data[3] = 4.1276

#     #Create a AudioData object
#     class AudioData():
#         def __init__(self ,data=None):
#             self.data = data.tobytes()
#             self.samples = len(data)

#     msg = AudioData(data)
   

#     # Call the message_to_numpy_array method
#     result = stt._message_to_numpy_array(msg)
#     stt.destroy_node()
#     rclpy.shutdown()

#     assert np.array_equal(result, data)

# def test_message_to_numpy_array_empty():
#     """
#     Tests the message_to_numpy_array method in the FasterWhisperLogic class.
#     Tests with an empty numpy array.
#     """
#     # Create a FasterWhisperLogic object
#     rclpy.init()
#     stt = STTNode()

#     data = np.ndarray(dtype=np.float32, shape=(0))
#     #data.shape(0)

#     #Create a AudioData object
#     class AudioData():
#         def __init__(self ,data=None):
#             self.data = data.tobytes()
#             self.samples = len(data)

#     msg = AudioData(data)

#     # Call the message_to_numpy_array method
#     with pytest.raises(ValueError) as exec_info:
#         result = stt._message_to_numpy_array(msg)
#     assert exec_info.type == ValueError

#     stt.destroy_node()
#     rclpy.shutdown()