# from speech_to_text.faster_whisper_node import STTNode
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from audio_data.msg import AudioData

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

    