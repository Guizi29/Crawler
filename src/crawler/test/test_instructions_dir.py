# Name of the program : test_instructions_dir.py
#
# Test node to test steering controls 
#
# Author: Guillaume Cren 
# Date: 24/07/2023

import sys
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Test_Instructions_Dir(Node):

    DIRECTION = "NONE"
    commands = {
        "w": "FORWARD",
        "s": "BACKWARD",
        "a": "LEFT",
        "d": "RIGHT", 
        " ": "STOP",
    }

    def __init__(self):
        super().__init__('test_instructions')
        self.instruction_publisher_ = self.create_publisher(String, 'instruction', 10)  
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]: #We define the keyboard as an input device in order to retrieve the value of the keyboard key
                input_char = sys.stdin.read(1)
                for key in self.commands.keys():
                    if str(input_char) == key:
                        self.DIRECTION = self.commands[key]
                        self.publish_direction()
    
    def publish_direction(self):
        msg = String()
        msg.data = self.DIRECTION
        self.instruction_publisher_.publish(msg)
        self.get_logger().info('Published instruction: %s' % msg.data)
        
def main(args=None):
    rclpy.init(args=args)
    test_dir = Test_Instructions_Dir()
    rclpy.spin(test_dir)
    test_dir.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
