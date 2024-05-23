"""
MIT License

Copyright (c) 2022 MUHAMMAD WAHAJ MURTAZA

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.



Additional modifications made by Johannes Eriksson, 2024:
Added publish_lidar_data()
Modified __init__()
Modified read_serial()
Modified read_range()
Modified start_loop()
Modified main()
"""

import serial
from time import sleep
import threading

import rclpy
from rclpy.node import Node
from lidar_data.msg import LidarData




class Lidar(Node):
    """
    A ROS2 node for transmitting lidar data.

    This node initializes a publisher to publish lidar data and captures lidar data from the sensor.

    Attributes: 

    Methods: 
        __init__ : Initializes the ros node, as well as the publisher and capturer of lidar data
        publish_lidar_data: publishes lidar data to a ROS2 topic
        read_serial: reads serial data from port
        read_range: reads parameters from serial data and stores it
        start: starts the lidar
        stop: stops the lidar
        start_loop: start the lidar loop
        terminate: stops and terimnate the process
    """

    distance = [0] * 360
    confidence = [0] * 360
    keep_loop = True

    def __init__(self, port, angle_offset=0):
        """
        Initializes the Lidar Node

        This method initiate Lidar object, aquire serial port, create ROS2 topic and start a lidar thread
        
        Args:
            port: Serial Port at which lidar is connected
            angle_offset: Angle Offset (to adjust the front at zero) { 0 > angle_offset < 360 }

        Returns:
             None
        """

        super().__init__("lidar")   

        self.ser = serial.Serial(port=port, baudrate=115200)
        self.angle_offset = angle_offset
        self.thread = threading.Thread(target=self.start_loop)

        self.publisher_ = self.create_publisher(LidarData, 'lidar/data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_lidar_data)

        self.thread.start()



    def publish_lidar_data(self):
        """
        Publish lidar data

        This method publishes the lidar data array distance and the length of the array to a ros2 topic

        Args:
            None

        Returns:
             None
        """
        msg = LidarData()
        msg.data = self.distance
        msg.length = len(self.distance)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


    def read_serial(self):
        """
        Reads serial data

        This method reads serial data (47 bytes) from port

        Args:
            None

        Returns:
             serial data
        """
        data = self.ser.read(47)
        return data


    def read_range(self, data):
        """
        Reads parameters from data

        This method reads parameters from serial data (47 bytes) and stores the distance 
        points and confidence of the lidar in arrays

        Args:
            data: serial data

        Returns:
             None
        """

        bytes_data = list(data)
        length = bytes_data[1]
        speed = (bytes_data[3] << 8) | bytes_data[2]
        start_angel = (bytes_data[5] << 8) | bytes_data[4]
        end_angel = (bytes_data[43] << 8) | bytes_data[42]
        time_stamp = (bytes_data[45] << 8) | bytes_data[44]
        crc = bytes_data[1]
    
        
        for i in range(11):       
            distance = (bytes_data[4 + (i*3)+3] << 8) | (bytes_data[3 + (i*3)+3])
            confidence = (bytes_data[5 + (i*3)+3]) 
            startangle = start_angel/100
            endangle = end_angel/100            
            angle_offsetted = (endangle + (i*(endangle - startangle)/11)) % 360
            self.distance[round(angle_offsetted)-1] = distance
            self.confidence[round(angle_offsetted)-1] = confidence
       

    def start(self):
        """
        Start the lidar

        This method starts the lidar

        Args:
            None

        Returns:
             None
        """
        self.ser.write(b'b')

    def stop(self):
        """
        Stop the lidar

        This method stop the lidar, can be started again

        Args:
            None

        Returns:
             None
        """
        self.ser.write(b'e')

    def start_loop(self):
        """
        Start the lidar loop

        This method start the lidar loop (must be called in a separate thread) 

        Args:
            None

        Returns:
             None
        """
        while self.keep_loop:
            data = self.read_serial()
            
            if data[0] != 84:
                self.ser.write(b'e')
                self.ser.close()
                sleep(0.1)
                self.ser.open()
                self.ser.write(b'b')
                continue

            self.read_range(data)
            

    def terminate(self):
        """
        Stop and erminate the process 

        This method stop the lidar, close serial port, and terminate thread
        Args:
            None

        Returns:
             None
        """
        self.stop()
        self.keep_loop = False
        self.ser.close()
        self.thread.join()

def main(args=None):
    
    rclpy.init(args=args)
    lidar = Lidar("/dev/ttyUSB0", angle_offset=0)
    lidar.start()
    rclpy.spin(lidar)
    rclpy.shutdown()

 

if __name__ == '__main__':
    main()
    