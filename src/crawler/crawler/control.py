import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from crawler import Crawler 
from motor import Motor 

class Control(Node):
    
    def __init__(self):
        super().__init__('control_node')   
        self.CR = Crawler()
        self.CR.PWM(1)
        self.CR.on_off(1)
        self.subscription = self.create_subscription(
            String,
            'instruction',
            self.control_callback,
            10
        )
        
    def control_callback(self, msg):
        self.get_logger().info("Message received: %s" %msg.data)
        if msg.data == 'LEFT':
            self.move_left()
        elif msg.data == 'RIGHT':
            self.move_right()
        elif msg.data == 'FORWARD':
            self.move_front()
        elif msg.data == 'BACKWARD':
            self.move_back()
        elif msg.data == 'STOP':
            self.stop()
        
    def move_left(self):
        self.CR.left(75)
    
    def move_right(self):
        self.CR.right(75)
        
    def move_front(self):
        self.CR.forward(75)
 
    def move_back(self):
        self.CR.backward(75)
        
    def stop(self):
        self.CR.forward(0)

def main(args=None):
    rclpy.init(args=args)
    control = Control()
    rclpy.spin(control)
    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
