from speech_to_text.faster_whisper_node import STTNode
from audio_data.msg import AudioData
from aida_interfaces.srv import SetState
import numpy as np
import rclpy
import pytest
import librosa
from pathlib import Path
import os


@pytest.fixture
def node():
    rclpy.init()
    yield STTNode()
    rclpy.shutdown()

def test_node_creation(node):
    assert node is not None

def test_publisher_creation_success(node):
    """
    Tests the creation of the publisher in the STTNode class.
    """
    assert node.publisher is not None
    created_topic = node.get_topic_names_and_types()
    assert ('/stt/stt_result', ['std_msgs/msg/String']) in created_topic

def test_service_creation_success(node):
    """
    Tests the creation of the service in the STTNode class.
    """
    assert node.srv is not None
    created_service = node.get_service_names_and_types()
    assert ('/stt/SetState', ['aida_interfaces/srv/SetState']) in created_service

def test_message_to_numpy_array(node):
    """
    Tests the message_to_numpy_array method in the FasterWhisperLogic class.
    Tests with a numpy array with 4 elements.
    """

    data = np.ndarray(shape=(4,), dtype=np.float32)
    data[0] = 1.32
    data[1] = 2.44
    data[2] = 3.14
    data[3] = 4.1276
    msg = AudioData()
    msg.data = data.tobytes()
    msg.samples = len(data)    
    
    # Call the message_to_numpy_array method
    result = node._message_to_numpy_array(msg)

    assert np.array_equal(result, data)

def test_message_to_numpy_array_empty(node):
    """
    Tests the message_to_numpy_array method in the FasterWhisperLogic class.
    Tests with an empty numpy array.
    """
    data = np.empty(dtype=np.float32, shape=(0,))
    
    #data = np.ndarray(dtype=np.float32, shape=(0))
    
    msg = AudioData()
    msg.data = data.tobytes()
    msg.samples = len(data)
    # Call the message_to_numpy_array method
    with pytest.raises(ValueError) as exec_info:
        result = node._message_to_numpy_array(msg)
    assert exec_info.type == ValueError

def test_transcribe_audio(node):
    """
    Tests the translate method for the Node.
    Tests with a audio file faster_whisper was trained on.
    """

    current_path = Path(os.path.dirname(os.path.realpath(__file__)))
    audio_path = current_path / "test_resource" / "jfk.wav"

    audio_data, _ = librosa.load(audio_path, sr=16000)
    
    # Call the transcribe_audio method
    result = node.translate(audio_data)
    assert result ==  "And so my fellow Americans, ask not what your country can do for you, ask what you can do for your country."

def test_set_state_of_node_idle(node):
    """
    Tests the set_state_of_node method in the Node.
    Tests with a request to stop trancription.
    """
    request = SetState.Request()
    request.desired_state = "idle"
    response = SetState.Response()
    node.set_state_of_node(request, response)
    
    assert node.active == False
    assert response.success == True
    assert response.message == "Successfully set state to: idle"
    # Reset back to default state 
    node.active = True

def test_set_state_of_node_active(node):
    """
    Tests the set_state_of_node method in the Node.

    Tests with a request to start the transcription.
    """

    request = SetState.Request()
    request.desired_state = "active"
    response = SetState.Response()
    node.set_state_of_node(request, response)
    assert node.active == True
    assert response.success == True
    assert response.message == "Successfully set state to: active"
    # Reset back to default state
    node.active = True
