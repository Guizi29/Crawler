import qwiic_icm20948
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Calibrator(Node):

    myIMU = qwiic_icm20948.QwiicIcm20948()
    NORTH_x = 0
    SOUTH_x = 0
    NORTH_y = 0
    SOUTH_y = 0
    MOVEMENT = "RIGHT"

    def __init__(self):
        super().__init__('calibrator')
        self.instruction_publisher_ = self.create_publisher(String, 'instruction', 10)  
        self.compass_publisher_ = self.create_publisher(String, 'orientation', 10)
        self.publish_instruction()
        self.calibration()
        self.publish_instruction()
        self.publish_compass()
        rclpy.shutdown()
              
    def calibration(self):
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
        self.MOVEMENT = "STOP"
            
    def publish_instruction(self):
        msg = String()
        msg.data = self.MOVEMENT
        self.instruction_publisher_.publish(msg)
        self.get_logger().info('Instruction published: "%s"' % msg.data)
            
    def publish_compass(self):
        msg = String()
        msg.data = str([self.NORTH_x, self.SOUTH_x, self.NORTH_y, self.SOUTH_y])
        self.compass_publisher_.publish(msg)
        self.get_logger().info('Compass published: "%s"' % msg.data)
     
def main(args=None):
    rclpy.init(args=args)
    open("/home/odroid/ros2_ws/src/crawler/resource/donnees.csv", "w").close()
    calibrator = Calibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()    
