import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from lidar_data.msg import LidarData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge                       


class LidarToImage(Node):
    distance = [0] * 360
    length = 0

    def __init__(self):
        super().__init__('lidar_to_image')
        self.bridge = CvBridge()
        self.frame_count = 0
        self.publisher = self.create_publisher(Image, 'lidar/image', 10)
        self.subscribe_to_lidar()
        #self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Subscribing: "%s"' % msg.data) 
        self.distance = msg.data
        self.lenght = msg.length

        canvas_width = 640
        canvas_height = 480
        canvas = np.full((canvas_height, canvas_width, 3), 255, dtype=np.uint8)  # Create white canvas

        # Sample distance list (adjust values as needed)
        distances = [100 for x in range(360)]  # 360 points at radius 100

        self.draw_points_on_canvas(canvas, distances)
        self.frame_count =+ 1
        self.publish_lidar_image(canvas)
        
            
    def subscribe_to_lidar(self):
        self.subscription = self.create_subscription(LidarData,'lidar/data',self.listener_callback,10)
         
    def publish_lidar_image(self, cv_img):
        msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
        msg.header.frame_id = str(self.frame_count)
        self.publisher.publish(msg)

    def draw_points_on_canvas(self,canvas, distances, color=(0, 0, 0), point_size=1):
        """Draws points on a canvas based on a list of distances from the center.

        Args:
            canvas: A NumPy array representing the white canvas (BGR format).
            distances: A list containing distances from the center for each point.
            color: A tuple representing the color of the points (default: blue).
            point_size: The size of the points (default: 3).

        Returns:
            None (modifies the canvas in-place).
        """

        # Calculate center coordinates
        center_x = int(canvas.shape[1] / 2)
        center_y = int(canvas.shape[0] / 2)

        # Ensure distances list length matches number of points
        if len(distances) != 360:
            print("Warning: Distance list length doesn't match 360 points. Using first", len(distances), "values.")
            distances = distances[:360]  # Truncate or pad distances list as needed

        # Normalize distances to fit canvas dimensions
        max_radius = min(center_x, center_y)  # Use the shorter dimension as max radius
        normalized_distances = [min(max_radius, d) for d in distances]

        # Draw points based on normalized distances and angles
        for angle in range(0, 360):
            radius = normalized_distances[angle]
            x = int(center_x + radius * np.cos(np.radians(angle)))
            y = int(center_y - radius * np.sin(np.radians(angle)))  # Subtract for upward Y-axis
            cv2.circle(canvas, (x, y), point_size, color, -1)






   



def main(args=None):
    rclpy.init(args=args)

    lidar_to_image = LidarToImage()

    rclpy.spin(lidar_to_image)

    lidar_to_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()