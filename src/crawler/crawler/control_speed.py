import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from crawler import Crawler 
from motor import Motor 

class Control_Speed(Node):
    
    def __init__(self):
        super().__init__('control_speed')   
        self.CR = Crawler()
        self.CR.PWM(1)
        self.CR.on_off(1)
        self.subscription = self.create_subscription(
            String,
            'speed',
            self.speed_callback,
            10
        )
        
    def speed_callback(self, msg):
        self.get_logger().info("Message received: %s" %msg.data)
        speed = int(msg.data)
        self.CR.speed_settings(speed)
        print(speed)

def main(args=None):
    rclpy.init(args=args)
    control_speed = Control_Speed()
    rclpy.spin(control_speed)
    control_speed.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
