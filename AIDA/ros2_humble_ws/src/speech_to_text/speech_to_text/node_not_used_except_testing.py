import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
    
    def addition(self, a, b) -> int:
        return a + b
    
def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    my_node.addition(3, 4)
    rclpy.shutdown()