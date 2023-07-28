# Name of the program : test_instructions_speed.py
#
# Test node for testing speed controls
#
# Author: Guillaume Cren 
# Date: 24/07/2023

import sys
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Test_Instructions_Speed(Node):

    SPEED = "NONE"

    def __init__(self):
        super().__init__('test_instructions_speed')
        self.instruction_publisher_ = self.create_publisher(String, 'speed', 10)  
        while True:
            self.SPEED = input("Enter speed (between 1 and 100)")
            self.publish_speed()
                
    def publish_speed(self):
        msg = String()
        msg.data = self.SPEED
        self.instruction_publisher_.publish(msg)
        self.get_logger().info('Published instruction: %s' % msg.data)
        
def main(args=None):
    rclpy.init(args=args)
    test_speed = Test_Instructions_Speed()
    rclpy.spin(test_speed)
    test_speed.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
