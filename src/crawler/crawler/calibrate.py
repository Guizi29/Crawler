# Name of the program : test_instructions_speed.py
#
# Test node for testing speed controls
#
# Author: Guillaume Cren 
# Date: 24/07/2023

import qwiic_icm20948
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Direction(Node):

    myIMU = qwiic_icm20948.QwiicIcm20948()
    NORTH_x = 0
    SOUTH_x = 0
    NORTH_y = 0 
    SOUTH_y = 0 
    X = 0
    Y = 0
    
    def __init__(self):
        super().__init__('calibrate')
        self.subscription = self.create_subscription(
            String,
            'compass',
            self.compass_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'orientation', 10)
            
    def compass_callback(self, msg):
        self.get_logger().info("Message received: %s" %msg.data)
        print(msg.data)
        coordinates = eval(msg.data)
        self.NORTH_x = coordinates[0]
        self.SOUTH_x = coordinates[1]
        self.NORTH_y = coordinates[2]
        self.SOUTH_y = coordinates[3]
        self.mesure_degrees()
        self.publish_direction()
                    
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
        if mag_y1-20 <= mag_y <= mag_y1+20:
            self.X = degree_x1
            self.Y = degree_y1
        if mag_y2-20 <= mag_y <= mag_y2+20:
            self.X = degree_x2
            self.Y = degree_y2
            
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
        
    def publish_direction(self):
        msg = String()
        msg.data = str([self.X, self.Y])
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    direction = Direction()
    rclpy.spin(direction)
    direction.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
