from audio.audio_transmit_mic import AudioTransmitterNode
from audio_data.msg import AudioData
from aida_interfaces.srv import SetState
import rclpy
import pytest
import numpy as np


@pytest.fixture
def node():
    rclpy.init()
    yield AudioTransmitterNode()
    rclpy.shutdown()

def test_node_creation(node):
    """
    Tests the creation of the STTNode class.
    """
    assert node is not None

def test_publisher_creation_success(node):
    """
    Tests the creation of the publisher in the STTNode class.
    """
    assert node.publisher is not None
    created_topic = node.get_topic_names_and_types()
    assert ('/mic/mic_audio', ['audio_data/msg/AudioData']) in created_topic

def test_service_creation_success(node):
    """
    Tests the creation of the service in the STTNode class.
    """
    assert node.srv is not None
    created_service = node.get_service_names_and_types()
    assert ('/mic/SetState', ['aida_interfaces/srv/SetState']) in created_service

def test_to_msg(node):
    """
    Tests the data_to_msg method in the AudioTransmitterNode class.
    Tests with a numpy array with 4 elements.
    """

    data = np.ndarray(shape=(4,), dtype=np.float32)
    data[0] = 1.32
    data[1] = 2.44
    data[2] = 3.14
    data[3] = 4.1276

    result = node._to_msg(data, 16000, 1)

    assert result.samples == len(data)
    assert result.sample_rate == 16000
    assert result.channels == 1
    assert np.array_equal(np.frombuffer(result.data, dtype=np.float32), data)

def test_to_msg_empty(node):
    """
    Tests the data_to_msg method in the AudioTransmitterNode class.
    Tests with an empty numpy array.
    """
    data = np.empty(dtype=np.float32, shape=(0,))
    result = node._to_msg(data, 16000, 1)

    assert result.samples == len(data)
    assert result.sample_rate == 16000
    assert result.channels == 1
    assert np.array_equal(np.frombuffer(result.data, dtype=np.float32), data)

def test_start_and_stop_workers(node):
    """
    Tests the start_workers method in the AudioTransmitterNode class.
    """
    node.start_workers()
    assert node.publisher_thread.is_alive()
    assert node.capturer_thread.is_alive()
    node.stop_workers()
    assert not node.publisher_thread.is_alive()
    assert not node.capturer_thread.is_alive()

def test_set_recording(node):
    """
    Tests the set_recording method in the AudioTransmitterNode class.
    """
    set_state = SetState.Request()
    set_state.desired_state = "active"
    response = SetState.Response()


    node.set_recording(set_state, response)
    assert response.success == True
    assert response.message == "Successfully set state to: active"
    
    assert node.publisher_thread.is_alive()
    assert node.capturer_thread.is_alive()

    set_state.desired_state = "idle"
    response = SetState.Response()
    node.set_recording(set_state, response)

    assert response.success == True
    assert response.message == "Successfully set state to: idle"
    assert not node.publisher_thread.is_alive()
    assert not node.capturer_thread.is_alive()