import qwiic_icm20948
import rclpy
import time
import signal
import matplotlib.pyplot as plt
import os
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

class Direction(Node):
   
    myIMU = qwiic_icm20948.QwiicIcm20948()
    NORTH_x = 0 
    SOUTH_x = 0
    NORTH_y = 0
    SOUTH_y = 0

    def __init__(self):
        super().__init__('direction_node')
        self.calibrate()
        print([self.NORTH_x, self.SOUTH_x])
        print([self.NORTH_y, self.SOUTH_y])
        while True : 
            self.X,self.Y = self.mesure_degrees()
            print([self.X, self.Y])
            time.sleep(5)

    def mesure_degrees(self): 
        self.myIMU.getAgmt()
        mag_x = self.myIMU.mxRaw
        print(mag_x)
        mag_y = self.myIMU.myRaw
        degree_x1, degree_x2 = self.from_mag_to_degrees(mag_x)
        print(degree_x1, degree_x2)
        degree_y1 = degree_x1 + 90 
        degree_y2 = degree_x2 + 90 
        print(degree_y1, degree_y2)
        degree_y1 = self.sup_to_360(degree_y1)
        degree_y2 = self.sup_to_360(degree_y2)
        print(degree_y1, degree_y2)
        if 0 <= degree_y1 <= 180:
            mag_y1 = self.from_degrees_to_mag_0_180(degree_y1)
        else:
            mag_y1 = self.from_degrees_to_mag_180_360(degree_y1)
        if 0 <= degree_y2 <= 180:
            mag_y2 = self.from_degrees_to_mag_0_180(degree_y2)
        else:
            mag_y2 = self.from_degrees_to_mag_180_360(degree_y2)
        print(mag_y1, mag_y2)
        print(mag_y)
        X = 0
        Y = 0 
        if mag_y1-20 <= mag_y <= mag_y1+20:
            X = degree_x1
            Y = degree_y1
        if mag_y2-20 <= mag_y <= mag_y2+20:
            X = degree_x2
            Y = degree_y2
        return X, Y
    
    def from_mag_to_degrees(self, mag):
        degree_1 = (mag - self.NORTH_x)/((self.SOUTH_x-self.NORTH_x)/180) # 
        degree_2 = (mag - self.SOUTH_x)/((self.NORTH_x-self.SOUTH_x)/180) + 180 #
        return degree_1, degree_2
    
    def from_degrees_to_mag_0_180(self, degree):
        mag_1 = degree*(self.SOUTH_y - self.NORTH_y)/180 + self.NORTH_y #
        return mag_1

    def from_degrees_to_mag_180_360(self, degree): 
        mag_2 = (degree-180)*(self.NORTH_y - self.SOUTH_y)/180 + self.SOUTH_y #
        return mag_2

    def sup_to_360(self, degree):
        if degree >= 360:
            degree = degree - 360 
            return degree 
        else:
            return degree      
        
    def calibrate(self):
        with open("/home/odroid/ros2_ws/src/crawler/resource/donnees.csv", "a") as fichier:
            for i in range(1,41):
                self.myIMU.getAgmt()
                fichier.write(
                f"{self.myIMU.mxRaw} {self.myIMU.myRaw}\n"
                )
                time.sleep(1)
        with open("/home/odroid/ros2_ws/src/crawler/resource/donnees.csv", 'r') as file:
            x_values = []
            y_values = []
            for line in file:
                data = line.strip().split()
                x_values.append(float(data[0]))
                y_values.append(float(data[1]))
            self.NORTH_x = max(x_values)
            self.SOUTH_x = min(x_values)
            self.NORTH_y = max(y_values)
            self.SOUTH_y = min(y_values)


def signal_handler(sig, frame):
    rclpy.shutdown()

def main(args=None):
    
    rclpy.init(args=args)
    open("/home/odroid/ros2_ws/src/crawler/resource/donnees.csv", "w").close()

    direction = Direction()

    signal.signal(signal.SIGINT, signal_handler)

    rclpy.spin(direction)

    direction.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()