import os
import config 
import rclpy
import signal
import sys
from rclpy.node import Node 
from time import sleep
from datetime import timedelta
import datetime
from crawler import Crawler 
from motor import Motor 
from std_msgs.msg import String

def signal_handler(sig, frame):
    rclpy.shutdown()

class Test(Node):

    DIRECTION = "FRONTWARD"
    SPEED = "0"
    
    def __init__(self):
        super().__init__('test_node')
        self.CR = Crawler()
        self.test_init_on()
        self.direction_publisher_ = self.create_publisher(String, 'direction', 10)  # Remplacez 'topic_name' par le nom du topic souhaité
        self.timer_ = self.create_timer(1.0, self.publish_direction)
        self.speed_publisher_ = self.create_publisher(String, 'speed', 10)  # Remplacez 'topic_name' par le nom du topic souhaité
        self.timer_ = self.create_timer(1.0, self.publish_speed)
        #sleep(10)
        self.test_move_front()
        self.test_move_back()
        self.test_move_left()
        self.test_move_right()
        self.test_init_off()
        rclpy.shutdown()

    def test_init_on(self):
        print("##################ACTIVATION OF THE INIT TEST##################")
        self.CR.PWM(1)
        self.CR.on_off(1)

    def test_init_off(self):
        print("##################DISABLING THE INIT TEST##################")
        self.CR.PWM(0)
        self.CR.on_off(0)

    def test_move_front(self):
        print("##################ACTIVATION OF THE FRONT MOVE TEST##################")
        self.DIRECTION = "FRONTWARD" 
        self.CR.forward(75)
        self.SPEED = "15000000"
        sleep(5)
        self.CR.forward(0)
        self.SPEED = "0"
        sleep(5)

    def test_move_back(self):
        print("##################ACTIVATION OF THE BACK MOVE TEST##################")
        self.DIRECTION = "BACKWARD" 
        self.CR.backward(75)
        self.SPEED = "15000000"
        sleep(5)
        self.CR.backward(0)
        self.SPEED = "0"
        sleep(5)

    def test_move_left(self):
        print("##################ACTIVATION OF THE LEFT MOVE TEST##################")
        self.DIRECTION = "LEFT" 
        self.CR.left(75)
        self.SPEED = "15000000"
        sleep(5)
        self.CR.left(0)
        self.SPEED = "0"
        sleep(5)

    def test_move_right(self):
        print("##################ACTIVATION OF THE RIGHT MOVE TEST##################")
        self.DIRECTION = "RIGHT" 
        self.CR.right(75)
        self.SPEED = "15000000"
        sleep(5)
        self.CR.right(0)
        self.SPEED = "0"
        sleep(5)
    
    def publish_direction(self): 
        msg = String()
        msg.data = self.DIRECTION
        print("Publishing direction:", self.DIRECTION)
        self.direction_publisher_.publish(msg)
        self.get_logger().info('Published direction: %s' % msg.data)
    
    def publish_speed(self):
        msg = String()
        msg.data = self.SPEED
        print("Publishing speed:", self.SPEED)
        self.speed_publisher_.publish(msg)
        self.get_logger().info('Published speed: %s' % msg.data)


def main(args=None):

    rclpy.init(args=args)

    test = Test()
    
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.spin(test)
    
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
