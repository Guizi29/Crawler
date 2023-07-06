#!/bin/bash

##################--SCRIPTS FOR MOTOR.PY--##################

# Script for init_GPIO function 
# $0 = script_path
# $1 = "section1"
# $2 = GPIO_number
if [ "$1" == "section1" ]; then
    sudo chown -R odroid:odroid /sys/class/gpio
    echo $2 > /sys/class/gpio/export
    sudo chown -R odroid:odroid /sys/class/gpio/gpio$2/*
    echo out > /sys/class/gpio/gpio$2/direction
fi

# Script for init_PWM_2 function 
# $0 = script_path
# $1 = "section2"
# $2 = PWM_number
if [ "$1" == "section2" ]; then
    sudo chown -R odroid:odroid /sys/class/pwm
    echo "$2" | sudo tee /sys/class/pwm/pwmchip0/export > /dev/null #pin 12 motor Right, pin 15 motor Left
    #PWM EXPORTED
    sudo chown -R odroid:odroid /sys/class/pwm/pwmchip0/*
    echo "20000000" | sudo tee /sys/class/pwm/pwmchip0/pwm$2/period > /dev/null
    #PWM PERIOD SET
fi

# Script for enable_PWM function 
# $0 = script_path
# $1 = "section3"
# $2 = on_off
# $3 = PWM_number
if [ "$1" == "section3" ]; then
    echo "$2" | sudo tee /sys/class/pwm/pwmchip0/pwm$3/enable > /dev/null
fi 

# Script for DIR function 
# $0 = script_path
# $1 = "section4"
# $2 = GPIO_number
# $3 = direction
if [ "$1" == "section4" ]; then
    echo "$3" | sudo tee /sys/class/gpio/gpio$2/value > /dev/null
fi 

# Script for IO2 function 
# $0 = script_path
# $1 = "section5"
# $2 = GPIO_number
# $3 = motor_on_off
if [ "$1" == "section5" ]; then
    echo "$3" | sudo tee /sys/class/gpio/gpio$2/value > /dev/null
fi 

# Script for duty_cycle function (part 1)
# $0 = script_path
# $1 = "section6"
# $2 = "enable_pwm" or "disable_PWM"
# $3 = PWM_number
if [ "$1" == "section6" ] && [ "$2" == "disable_PWM" ]; then
    echo 0 > /sys/class/pwm/pwmchip0/pwm$3/enable
elif [ "$1" == "section6" ] && [ "$2" == "enable_PWM" ]; then
    echo 1 > /sys/class/pwm/pwmchip0/pwm$3/enable
else 
    true 
fi

# Script for duty_cycle function (part 2)
# $0 = script_path
# $1 = "section6"
# $2 = duty_cycle
# $3 = PWM_number
if [ "$2" == "section6" ]; then 
    echo "$1" | sudo tee /sys/class/pwm/pwmchip0/pwm$3/duty_cycle
fi 

