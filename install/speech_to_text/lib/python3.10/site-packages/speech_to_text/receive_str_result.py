import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ReceiveSTTResult(Node):
    """
    Class for receiving and printing the result from STT model.

    Attributes: 
        subscription: The subscription object for receiving audio data
    Methods:
        __init__ : For initializing the ReceiveSTTResult node
        listerer_callback : The callback function for output received data from topic
    """

    def __init__(self):
        """
        For initializing the ReceiveSTTResult node

        Args:
            None
        
        Returns:
            None
        """

        super().__init__('receive_stt_result', namespace='stt')
        self.subscription = self.create_subscription(
            String,
            'stt_result',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        The callback function for output received data from topic
        
        Args:
            msg: A string object representing the result from STT model
        
        Returns:
            None
        """

        self.get_logger().info('STT result :' + msg.data)


def main(args=None):
    rclpy.init(args=args)

    stt_result_node = ReceiveSTTResult()
    try:
        rclpy.spin(stt_result_node)
    except KeyboardInterrupt:
        stt_result_node.get_logger().info('STT result: Keyboard interrupt')

    stt_result_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
