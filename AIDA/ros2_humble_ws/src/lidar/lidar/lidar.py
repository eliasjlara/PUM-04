"""
This is code to interact lidar: HLS-LFCD2 with python3.
:author: Wahaj Murtaza
:github: https://github.com/wahajmurtaza
:email: wahajmurtaza@gmail.com
"""

import serial
from time import sleep
import threading
import tkinter as tk
import math


import rclpy
from rclpy.node import Node
from lidar_data.msg import LidarData



# LIDAR DATASHEET: https://emanual.robotis.com/assets/docs/LDS_Basic_Specification.pdf
# RED       5V
# BROWN     TX
# ORANGE    PWM     (connect with pwm) # ground internally
# BLACK     GND
# GREEN     RX
# BLUE      BOT     (not used)

# BLACK     PWM     (connect with pwm)
# RED       5V


class Lidar(Node):
    distance = [0] * 360
    confidence = [0] * 360
    rpm = 0
    keep_loop = True

    def __init__(self, port, angle_offset=0):
        super().__init__("lidar")   

        """
        Initiate Lidar object, aquire serial port, set parameters and start a lidar thread
        :param port: Serial Port at which lidar is connected
        :param angle_offset: Angle Offset (to adjust the front at zero) { 0 > angle_offset < 360 }
        """

        self.ser = serial.Serial(port=port, baudrate=115200)
        self.angle_offset = angle_offset
        self.thread = threading.Thread(target=self._start_loop)

        self.publisher_ = self.create_publisher(LidarData, 'lidar/data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.counterr = 0
        # Instansiera TKinter
        #root = tk.Tk()
        #self.canvas = tk.Canvas(root, width=600, height=600)
        #self.canvas.pack()

        self.thread.start()
        #root.mainloop()
        #self.counter_ = 0

      

    def timer_callback(self):
        msg = LidarData()
        #msg.header.frame_id = 10
        msg.data = self.distance
        msg.length = len(self.distance)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def _read_serial(self):
        """
        read serial data (42 bytes)
        :return: serial data
        """
        data = self.ser.read(47)
        #print("1")
        #print(data)
        return data

    def _read_range(self, data):
        """
        read parameters from data
        :param data: serial data
        :return: None
        """
        bytes_data = list(data)
        #print(bytes_data)
        #print("Hello")
        degree = (bytes_data[1] - 0xA0) * 6
        self.rpm = (bytes_data[3] << 8) | bytes_data[2]

        if bytes_data[41] != bytes_data[40]  or bytes_data[40] == 0:
           # print(f'invalid data: {degree}')
            return

        for i in range(6):
            distance = (bytes_data[2 + (i*4)+3] << 8) | (bytes_data[2 + (i*4)+2])
            intensity = (bytes_data[2 + (i*4)+1] << 8) | (bytes_data[2 + (i*4)+0])
            angle = degree + i
            angle_offsetted = angle + self.angle_offset if angle + self.angle_offset < 360 else angle + self.angle_offset - 360
            self.distance[angle_offsetted] = distance
            self.intensity[angle_offsetted] = intensity


    def _new_read_range(self, data):
        bytes_data = list(data)

        length = bytes_data[1]
       # print("this is length " + str(length))

        speed = (bytes_data[3] << 8) | bytes_data[2]
        #print("this is speed " + str(speed))

        start_angel = (bytes_data[5] << 8) | bytes_data[4]
        #print("this is start_angel " + str(start_angel/100))

        end_angel = (bytes_data[43] << 8) | bytes_data[42]
        #print("this is end_angel " + str(end_angel/100))

        time_stamp = (bytes_data[45] << 8) | bytes_data[44]
        #print("this is time_stamp " + str(time_stamp))

        crc = bytes_data[1]
       # print("this is crs " + str(crc))
        
        for i in range(11):
            
            distance = (bytes_data[4 + (i*3)+3] << 8) | (bytes_data[3 + (i*3)+3])
            confidence = (bytes_data[5 + (i*3)+3]) 
            startangle = start_angel/100
            endangle = end_angel/100
            #angle_offsetted = angle + self.angle_offset if angle + self.angle_offset < 360 else angle + self.angle_offset - 360
            angle_offsetted = (endangle + (i*(endangle - startangle)/11)) % 360
          #  print("this is start angle " + str(startangle))
           # print("this is end angle " + str(endangle))
            #print(str(i) +" this is angle offsetted " + str(angle_offsetted))
            self.distance[round(angle_offsetted)-1] = distance
            self.confidence[round(angle_offsetted)-1] = confidence
        
       

       # print("this is distance " + str(self.distance))
        #print("this is confidence " + str(self.confidence))
       

    def start(self):
        """
        start the lidar
        :return: None
        """
        self.ser.write(b'b')

    def stop(self):
        """
        stop the lidar, can be started again
        :return: None
        """
        self.ser.write(b'e')

    def _start_loop(self):
        """
        Start the lidar loop (must be called in a separate thread)
        :return: None
        """
        counter = 0
        while self.keep_loop:
            data = self._read_serial()
           # print(data)
            #print(data[0])
            
            
            if data[0] != 84:
                self.ser.write(b'e')
                self.ser.close()
                sleep(0.1)
                self.ser.open()
                self.ser.write(b'b')
                continue
            

            #print("Hello")
            #sleep(1)
            self._new_read_range(data)
            
            #if counter == 100:
             #   counter = 0
              #  self.draw_lidar_data(self.distance, self.confidence)
            #counter += 1

    def terminate(self):
        """
        stop the leader, close serial port, and terminate thread
        :return: None
        """
        self.stop()
        self.keep_loop = False
        self.ser.close()
        self.thread.join()

    def get_distance(self):
        """
        get  distace
        :return: return distance array 1x360
        """
        return self.distance

    def get_intensity(self):
        """
        get lidar intensity
        :return: return intensity array 1x360
        """
        return self.intensity

    def get_rpm(self):
        """
        get lidar RPM
        :return: int
        """
        return self.rpm
    
    # Metod för att rita lidar-data 
    def draw_lidar_data(self, data, confidence):
        center_x, center_y = 300, 300  # Anta mitten av canvas
        max_radius = 250  # Hur långt ifrån mittpunkten datan ritas
        

        self.canvas.delete("all")
        for i, distance in enumerate(data):
            if(confidence[i]) < 220:
                continue
            angle_radians = math.radians(i)
            x = center_x + (distance/(max_radius*10)) * max_radius * math.cos(angle_radians)  
            y = center_y + (distance/(max_radius*10)) * max_radius * math.sin(angle_radians)  
            self.canvas.create_oval(x-1, y-1, x+1, y+1, fill="black") 

    # Hämtar lidar-data (exempeldata, ska bytas mot riktig sensor)
    # Ritar/Uppdaterar canvas


def main(args=None):
    
    rclpy.init(args=args)
    lidar = Lidar("/dev/ttyUSB0", angle_offset=0)
    lidar.start()
    rclpy.spin(lidar)
    rclpy.shutdown()



    a = 0
    while(True):
        a+=1        
        sleep(2)

    #lidar.stop()

    #lidar.terminate()
 

if __name__ == '__main__':
    main()
    
