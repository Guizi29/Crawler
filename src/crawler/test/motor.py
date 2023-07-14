
# Name of the program : init_motor.py
#
# Create a node to initialiaze the function of the crawler like PWM, GPIO or IO2
#
# Author: Guillaume Cren & Marie Le Bris
# Date: 14/06/2023

import rclpy
from rclpy.node import Node
import os
import subprocess

class Motor(Node):

    script_path = "../resource/scripts.sh" 
    ## The constructor.
    #  @param self The object pointer.
    #  @param GPIO_DIR: GPIO number which manages the direction of rotation of the motor
    #  @type GPIO_DIR: int
    #  @param GPIO_IO2 : GPIO number that manages whether or not the motir is activated
    #  @type GPIO_IO2: int
    #  @param PWM: number of the PWM
    #  @type PWM: int
    def __init__(self, GPIO_IO2, GPIO_DIR, PWM):
        super().__init__('motor_node')
        self.GPIO_DIR_number = GPIO_DIR
        self.GPIO_IO2_number = GPIO_IO2
        self.PWM_number = PWM
        self.get_logger().info('MOTOR initialized')

    #############################################--INIT METHOD GPIO--#############################################
    ## Documentation for a method.
    #  initializes a GPIO
    #  @param self The object pointer.
    #  @param GPIO_number: GPIO number to initialized 
    #  @type GPIO_number: int
    def init_GPIO(self, GPIO_number):
        print("-----> Init GPIO %d" % GPIO_number)
        if os.path.exists("/sys/class/gpio/gpio" + str(GPIO_number) + "/direction") == False:
            try:
                subprocess.call([self.script_path, "section1", str(GPIO_number)])
                print("-----> GPIO %d initialized" % GPIO_number)
            except subprocess.CalledProcessError as e:
                # The execution of the command with subprocess above has generated an error 
                print(f"Erreur lors de l'exécution de la commande: {e}")
        else:
            print("-----> GPIO %d already initialized" % GPIO_number)

    #############################################--INIT METHOD PWM--#############################################
    ## Documentation for init_PWM_2 method.
    #  initializes the two PWMs
    #  warning: with odroid-C2 impossible to initialize a PWM then the second. Both are therefore initialized simultaneously
    #  @param self The object pointer.
    #  @param PWM_number: PWM number to be initialized
    #  @type PWM_number: int
    def init_PWM_2(self, PWM_number):
        print("-----> Init PWM %d" % PWM_number)
        if os.path.exists("/sys/class/pwm/pwmchip0/pwm"+str(PWM_number)+"/period") == False:
            try:
                subprocess.call([self.script_path, "section2", str(PWM_number)])
                print("-----> PWM %d initialized" % PWM_number)
            except subprocess.CalledProcessError as e:
                # The execution of the command with subprocess above has generated an error 
                print(f"Erreur lors de l'exécution de la commande: {e}")
        else:
            print("-----> PWM %d already initialized" % PWM_number)

    #############################################--ACTIVATE OR DESACTIVATE PWM RIGHT AND LEFT--#############################################
    ## Documentation for enable_PWM method.
    #  enable or disable a PWM
    #  @param self The object pointer.
    #  @param on_off: 1 for enable, 0 for disable
    #  @type on_off: int
    def enable_PWM(self, on_off, PWM_number):
        print("PWM ENABLE %d" %on_off)
        try:
            subprocess.call([self.script_path, "section3", str(on_off), str(PWM_number)])
            return True  # There is no error, return True
        except subprocess.CalledProcessError as e:
            # The execution of the command with subprocess above has generated an error 
            print(f"Erreur lors de l'exécution de la commande: {e}")
            return False  # There is an error, return False

    #############################################--DIR METHOD GPIO--#############################################
    ## Documentation for DIR method.
    #  determines the direction of rotation of the motor
    #  @param self The object pointer.
    #  @param GPIO_number: direction GPIO number (GPIO_DIR)
    #  @type GPIO_number: int
    #  @param direction: 0 for clockwise, 1 for trigonometric
    #  @type direction: int
    def DIR(self, GPIO_number, direction):
        print("-----> GPIO %d" % GPIO_number,"-->Direction %d" %direction)
        try:
            subprocess.call([self.script_path, "section4", str(GPIO_number), str(direction)])
            return True  # There is no error, return True
        except subprocess.CalledProcessError as e:
            # The execution of the command with subprocess above has generated an error 
            print(f"Erreur lors de l'exécution de la commande: {e}")
            return False  # There is an error, return False

    #############################################--ACTIVATE OR DESACTIVATE THE MOTOR--#############################################
    ## Documentation for IO2 method.
    #  enable or disable a motor
    #  @param self The object pointer.
    #  @param GPIO_number: GPIO number (GPIO_IO2)
    #  @type GPIO_number: int
    #  @param motor_on_off: 0 for disable, 1 for enable
    #  @type motor_on_off: int
    def IO2(self, GPIO_number, motor_on_off):
        print("-----> GPIO %d" % GPIO_number, "-->Value %d" %motor_on_off)
        try:
            subprocess.call([self.script_path, "section5", str(GPIO_number), str(motor_on_off)])
            return True  # There is no error, return True
        except subprocess.CalledProcessError as e:
            # The execution of the command with subprocess above has generated an error 
            print(f"Erreur lors de l'exécution de la commande: {e}")
            return False  # There is an error, return False

    #############################################--DUTY CYCLE METHOD PWM--#############################################
    ## Documentation for duty_cycle method.
    #  determines the motor speed
    #  @param self The object pointer.
    #  @param duty_cycle: percentage value of the motor speed (0 to 100)
    #  @type duty_cycle: int
    #  @¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡ OJO-->must have duty_cycle < period !!!!!!!!!!!!!!!! (period defined in init_PWM_2 method )
    #◘ @18000000 is the max period of PWM
    def duty_cycle(self, duty_cycle, PWM_number):
        if int(duty_cycle) == 0 :
                    if PWM_number == 0: 
                        print("I DISABLE PWM R BECAUSE DUTY=0")
                    else: 
                        print("I DISABLE PWM L BECAUSE DUTY=0")
                    try:
                        subprocess.call([self.script_path, "section6", "disable_PWM", str(PWM_number)])
                    except subprocess.CalledProcessError as e:
                        # The execution of the command with subprocess above has generated an error 
                        print(f"Erreur lors de l'exécution de la commande: {e}")
                        return False  # There is an error, return False
        else :
                    if PWM_number == 0: 
                        print("I ENABLE PWM R BECAUSE DUTY=0")
                    else: 
                        print("I ENABLE PWM L BECAUSE DUTY=0")
                    try:
                        subprocess.call([self.script_path, "section6", "enable_PWM", str(PWM_number)])
                    except subprocess.CalledProcessError as e:
                        # The execution of the command with subprocess above has generated an error 
                        print(f"Erreur lors de l'exécution de la commande: {e}")
                        return False  # There is an error, return False
        try:
            value_duty_cycle = int((float(duty_cycle) / 100) * 20000000)
            subprocess.call([self.script_path, str(value_duty_cycle), "section6", str(PWM_number)])
            return True  # There is no error, return True
        except subprocess.CalledProcessError as e:
            # The execution of the command with subprocess above has generated an error 
            print(f"Erreur lors de l'exécution de la commande: {e}")
            return False  # There is an error, return False

def main(args=None):
    rclpy.init(args=args)
    motor = Motor(GPIO_IO2, GPIO_DIR, PWM)
    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
