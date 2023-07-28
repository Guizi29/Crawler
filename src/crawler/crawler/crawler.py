# Name of the program : crawler.py
#
# Class that uses motor.py functions based on export numbers
#
# Author: Guillaume Cren & Marie Le Bris
# Date: 14/06/2023

import rclpy
from rclpy.node import Node
import config
import os
import subprocess
import sys
from motor import Motor

class Crawler(Node):

    ## The constructor.
    def __init__(self):
        super().__init__('crawler')
        self.MR = Motor(config.motor_right_IO2, config.motor_right_DIR, 0)
        self.ML = Motor(config.motor_left_IO2, config.motor_left_DIR, 1)
        self.init_IO2_DIR()
        self.init_PWM()
        #self.init_light()
        self.get_logger().info('All components are initialized, Crawler initialized')

    ## Documentation for init_I02_DIR method.
    #  initializes GPIOs for both motors
    #  @param self The object pointer.
    def init_IO2_DIR(self):
        self.MR.init_GPIO(config.motor_right_IO2)
        self.ML.init_GPIO(config.motor_left_IO2)
        self.MR.init_GPIO(config.motor_right_DIR)
        self.ML.init_GPIO(config.motor_left_DIR)
 
    ## Documentation for init_PWM method.
    #  @param self The object pointer.
    def init_PWM(self):
        self.MR.init_PWM_2(config.motor_right_PWM)
        self.ML.init_PWM_2(config.motor_left_PWM)
       
    ## Documentation for PWM method.
    #  enables or disables PWMs
    #  @param self The object pointer.
    #  @param on_off: 1 to activate PWM, 0 to deactivate PWM
    #  @type on_off: int
    def PWM(self, on_off):
        if self.MR.enable_PWM(on_off, config.motor_right_PWM) == True:
            if self.ML.enable_PWM(on_off, config.motor_left_PWM) == True:
                return True
            else:
                print("Error enabling PWM for motor_left_PWM")
                return False
        else:
            print("Error enabling PWM for motor_right_PWM")
            return False

    ## Documentation for on_off method.
    #  enables or disables motors
    #  @param self The object pointer.
    #  @param motor_on_off: 1 to activate motors, 0 to deactivate motors
    #  @type motor_on_off: int
    def on_off(self, motor_on_off):
        if self.MR.IO2(config.motor_left_IO2, motor_on_off) == True:
            if self.ML.IO2(config.motor_right_IO2, motor_on_off) == True:
                return True
            else:
                print("Error setting IO2 for motor_right_IO2")
                return False
        else:
            print("Error setting IO2 for motor_left_IO2")
            return False

    #############################################--DECLARATION OF THE DIRECTIONS--#############################################
    ## Documentation for forward method.
    #  moves forward the robot at variable speed
    #  warning: a speed lower than 50%, can cause a speed difference between the two motors
    #  @param self The object pointer.
    #  @param duty_cycle: percentage value of the robot speed (0 to 100)
    #  @type duty_cycle: int
    def forward(self):
        if self.MR.DIR(config.motor_right_DIR, int(0)) == True:
            if self.ML.DIR(config.motor_left_DIR, int(1)) == True:
                return True 
                self.get_logger().info('Moving backward')
            else:
                print("Error setting direction for motor_left_DIR")
                return False
        else:
            print("Error setting direction for motor_right_DIR")
            return False

    ## Documentation for backward method.
    #  moves backward the robot at variable speed
    #  warning: a speed lower than 50%, can cause a speed difference between the two motors
    #  @param self The object pointer.
    #  @param duty_cycle: percentage value of the robot speed (0 to 100)
    #  @type duty_cycle: int
    def backward(self):
        if self.MR.DIR(config.motor_right_DIR, 1) == True:
            if self.ML.DIR(config.motor_left_DIR, 0) == True:
                return True 
                self.get_logger().info('Moving backward')
            else:
                print("Error setting direction for motor_left_DIR")
                return False
        else:
            print("Error setting direction for motor_right_DIR")
            return False

    ## Documentation for right method.
    #  rotates the robot on itself to the right (clockwise) at variable speed
    #  @param self The object pointer.
    #  @param duty_cycle: percentage value of the robot speed (0 to 100)
    #  @type duty_cycle: int
    def right(self):
        if self.MR.DIR(config.motor_right_DIR, 0) == True:
            if self.ML.DIR(config.motor_left_DIR, 0) == True:
                return True 
                self.get_logger().info('Rotating right')
            else:
                print("Error setting direction for motor_left_DIR")
                return False
        else:
            print("Error setting direction for motor_right_DIR")
            return False

    ## Documentation for left method.
    #  rotates the robot on itself to the left (trigonometric direction) at variable speed
    #  @param self The object pointer.
    #  @param duty_cycle: percentage value of the robot speed (0 to 100)
    #  @type duty_cycle: int    
    def left(self):
        if self.MR.DIR(config.motor_right_DIR, 1) == True:
            if self.ML.DIR(config.motor_left_DIR, 1) == True:
                return True 
                self.get_logger().info('Rotating left')
            else:
                print("Error setting direction for motor_left_DIR")
                return False
        else:
            print("Error setting direction for motor_right_DIR")
            return False
            
    def speed_settings(self, duty_cycle):
        if self.MR.duty_cycle(duty_cycle, config.motor_right_PWM) == True:
            if self.ML.duty_cycle(duty_cycle, config.motor_left_PWM) == True:
                if duty_cycle == 0:
                    self.get_logger().info('We are stopped')
                else:
                    self.get_logger().info('We are moving !!')
                return True
            else:
                print("Error setting duty cycle for motor_left_PWM")
                return False
        else:
            print("Error setting duty cycle for motor_right_PWM")
            return False

    #############################################--SET UP FOR THE LIGHTS--#############################################
    ## Documentation for init_light method.
    #  unfinished function
    #  @param self The object pointer.
    def init_light(self):
        self.get_logger().info('There is a light that never goes out')
        self.MR.init_GPIO(config.light1)
        self.MR.init_GPIO(config.light2)
        self.MR.init_GPIO(config.light3)

    ## Documentation for init_I02_DIR method.
    #  unfinished function
    #  @param self The object pointer.
    #  @param on_off: 1 to activate light, 0 to deactivate light
    #  @type on_off: int
    def light_on_off(self, on_off):
        command1 = "echo "+ str(on_off) + " >/sys/class/gpio/gpio"+str(config.light1)+"/value"
        os.system(command1)
        command2 = "echo "+ str(on_off) + " >/sys/class/gpio/gpio"+str(config.light2)+"/value"
        os.system(command2)
        command3 = "echo "+ str(on_off) + " >/sys/class/gpio/gpio"+str(config.light3)+"/value"
        os.system(command3)

def main(args=None):
    rclpy.init(args=args)
    crawler = Crawler()
    rclpy.spin(crawler)
    crawler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
