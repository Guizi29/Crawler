import qwiic_icm20948
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Trajector(Node):

    directionUser = "none"
    X = "0"

    def __init__(self):
        super().__init__('trajector')
        subscription = self.create_subscription(
            String,
            'orientation',
            self.orientation_callback,
            10
            )
        instruction_publisher_ = self.create_publisher(String, 'instruction', 10)
        
    def orientation_callback(self, msg):

        I1 = []
        I2 = []
        I3 = []
        self.directionUser = input("Enter the instrcution for the trajectory")
        
        if self.directionUser == "none":
            pass 
        else:
            eval(msg.data)
            self.X = msg.data[0]
            int(self.X)
            minX = int(self.directionUser)-180 
            maxX = int(self.directionUser)+180
            if minX < 0 :
                #I1 = interval[directionUser-180+360, 360] #Crawler will be rotating left 
                #I2 = interval[0, directionUser] #Crawler will be rotating left
                #I3 = interval[directionUser, directionUser-180+360] #Crawler will be rotating right
                if 0 < self.X < int(self.directionUser) or int(self.directionUser)-180+360 < self.X < 360: 
                    while not int(self.directionUser)-10 < self.X < int(self.directionUser)+10:
                        publish_instruction("LEFT")
                else:
                    while not int(self.directionUser)-10 < self.X < int(self.directionUser)+10:
                        publish_instruction("RIGHT")
                self.directionUser = "none"
            if maxX > 360 :
                #I1 = interval[directionUser, 360] #Crawler will be rotating right
                #I2 = interval[0, directionUser-360+180] #Crawler will be rotating right
                #I3 = interval[directionUser-360+180, directionUser] #Crawler will be rotating left
                if 0 < self.X < int(self.directionUser)-360+180 or int(self.directionUser) < self.X < 360: 
                    while not int(self.directionUser)-10 < self.X < int(self.directionUser)+10:
                        publish_instruction("RIGHT")
                else:
                    while not int(self.directionUser)-10 < self.X < int(self.directionUser)+10:
                        publish_instruction("LEFT")
                self.directionUser = "none"
            
    def publish_instruction(self, direction):
        msg = String()
        msg.data = direction
        instruction_publisher_.publish(msg)
        get_logger().info('Published instruction: %s' % msg.data)
        
def main(args=None):
    rclpy.init(args=args)
    trajector = Trajector()
    rclpy.spin(trajector)
    trajector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()      
