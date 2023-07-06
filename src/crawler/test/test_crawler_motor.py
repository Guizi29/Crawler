import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from crawler import Crawler 
from motor import Motor 
from std_msgs.msg import String

class Test_CM(Node):
    
    def __init__(self):
        super().__init__('test_cm_node')
        self.CR = Crawler()
        self.test_init_on()
        self.test_move_front()
        self.test_move_back()
        self.test_move_left()
        self.test_move_right()
        self.test_init_off()

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
        time.sleep(5)
        self.CR.forward(0)
        self.SPEED = "0"
        time.sleep(5)

    def test_move_back(self):
        print("##################ACTIVATION OF THE BACK MOVE TEST##################")
        self.DIRECTION = "BACKWARD" 
        self.CR.backward(75)
        self.SPEED = "15000000"
        time.sleep(5)
        self.CR.backward(0)
        self.SPEED = "0"
        time.sleep(5)

    def test_move_left(self):
        print("##################ACTIVATION OF THE LEFT MOVE TEST##################")
        self.DIRECTION = "LEFT" 
        self.CR.left(75)
        self.SPEED = "15000000"
        time.sleep(5)
        self.CR.left(0)
        self.SPEED = "0"
        time.sleep(5)

    def test_move_right(self):
        print("##################ACTIVATION OF THE RIGHT MOVE TEST##################")
        self.DIRECTION = "RIGHT" 
        self.CR.right(75)
        self.SPEED = "15000000"
        time.sleep(5)
        self.CR.right(0)
        self.SPEED = "0"
        time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    test = Test_CM()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
