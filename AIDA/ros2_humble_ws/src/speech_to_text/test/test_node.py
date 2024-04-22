import rclpy
from speech_to_text.node_not_used_except_testing import MyNode
import pytest

@pytest.fixture
def node():
    rclpy.init()
    yield MyNode()
    rclpy.shutdown()

def test_node_creation(node):
    assert node is not None

def test_addition(node):
    assert node.addition(3, 4) == 7