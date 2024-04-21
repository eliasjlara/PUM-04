import pytest
from aida_api.aida_api.ros2_interface import InterfaceNode
from aida_interfaces.msg import Image
from std_msgs.msg import String

@pytest.fixture
def interface_node():
    api = InterfaceNode()
    yield api
    api.destroy_node()

def test_start_camera(interface_node):
    interface_node.start_camera()
    # Add your assertions here to verify the behavior of the start_camera method

def test_start_microphone(interface_node):
    interface_node.start_microphone()
    # Add your assertions here to verify the behavior of the start_microphone method

def test_stop_camera(interface_node):
    interface_node.stop_camera()
    # Add your assertions here to verify the behavior of the stop_camera method

def test_stop_microphone(interface_node):
    interface_node.stop_microphone()
    # Add your assertions here to verify the behavior of the stop_microphone method

def test_video_callback(interface_node):
    # Create a mock Image message
    msg = Image()

    # Call the video_callback method
    interface_node.video_callback(msg)

    # Assert that the message is added to the queue
    assert len(interface_node.video_queue) == 1
    assert interface_node.video_queue[0] == msg
    

def test_stt_callback(interface_node):
    # Create a mock String message
    msg = String()

    # Call the stt_callback method
    interface_node.stt_callback(msg)

    # Assert that the message is added to the queue
    assert len(interface_node.stt_queue) == 1
    assert interface_node.stt_queue[0] == msg

def test_destroy_node(interface_node):
    pass

def test_init_clients(interface_node):
    interface_node.init_clients()
    assert interface_node.camera_client is not None
    assert interface_node.mic_client is not None

def test_init_pubs(interface_node):
    interface_node.init_pubs()
    assert interface_node.joystick_pub is not None

def test_init_subs(interface_node):
    interface_node.init_subs()
    assert interface_node.camera_sub is not None
    assert interface_node.stt_sub is not None
    

def test_init_queues(interface_node):
    interface_node.init_queues()
    assert interface_node.video_queue == []
    assert interface_node.stt_queue == []

def test_start_workers(interface_node):
    interface_node.start_workers()
    assert interface_node.joystick_publisher_event is not None
    assert interface_node.joystick_publisher_thread is not None
    assert interface_node.server_event is not None
    assert interface_node.server_thread is not None

def test_stop_workers(interface_node):
    interface_node.start_workers()
    interface_node.stop_workers()
    assert interface_node.joystick_publisher_event.is_set()
    assert not interface_node.joystick_publisher_thread.is_alive()
    assert interface_node.server_event.is_set()
    assert not interface_node.server_thread.is_alive()

def test_to_joystick_msg(interface_node):
    pass

def test_publish_joystick(interface_node):
    pass

def test_start_server(interface_node):
    pass

def test_handle_client(interface_node):
    pass

def test_handle_message(interface_node):
    pass

def test_handle_camera(interface_node):
    pass

def test_handle_image_analysis(interface_node):
    pass

def test_handle_mic(interface_node):
    pass

def test_handle_stt(interface_node):
    pass

def test_handle_lidar(interface_node):
    pass

def test_handle_req_video_feed(interface_node):
    pass

def test_handle_req_stt(interface_node):
    pass

def test_handle_text(interface_node):
    pass

def test_handle_joystick_move(interface_node):
    pass

def test_send_video_stream(interface_node):
    pass

def test_send_video_frame(interface_node):
    pass

if __name__ == '__main__':
    pytest.main()
