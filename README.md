# Datasheet for the crawler 

Author : Guillaume Cren
Contact : crenguillaume29@gmail.com 
Date : 06/2023

Hardware :
Odroid C4 
Sensor IMU Sparkfun Qwiic 9DoF ICM20948
Speed controller Faulhaber SC5008s 3530

Software : 
Ubuntu 22.04 
ROS 2 Humble


## Purpose of the project

I carried out this project for OBSEA Research, a research center of the Polytechnic University of Catalonia based in Vilanova y La Geltru and specialized in the study of the seabed and the collection of oceanographic data, as part of my technical internship during my engineering studies. The aim of this robotics project was to operate a 2-wheeled underwater robot called the "crawler". This robot allows ...
My mission was to rework the system already present in the robot. The basic system was based on python libraries created by my predecessor and controlled via a Web API. Danial Toma, my tutor during this internship, asked me to convert this system to ROS 2 Humble and to integrate a new sensor into the system. 


## ROS2 workspace 

The first step is to install Ubuntu 22.04 Jammy to run ROS 2 Humble. 
Old version: Ubuntu 20.04 Focal.  
Once the upgrade is complete, you can install ROS 2 Humble according to the instructions on the ROS website (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). 
Once ROS 2 Humble is installed. You'll still need to source it each time you open a new terminal to run code using ROS2. 
The command is as follows :
source /opt/ros/humble/setup.bash

In order to run the crawler from an appropriate environment, a workspace has been created under the name ros2_ws.
It contains all the programs required to run the crawler.
This directory contains the ROS2 work tree. It contains all the code required to run the crawler. Here's the tree :


- ros2_ws/        
    - build/          
    - install/        	
    - log/            	
    - src/  	
        - crawler/  
            - crawler/
			- calibrate.py
			- calibrator.py
			- config.py
			- control_dir.py
			- control_speed.py
			- crawler.py
			- __init__.py
			- motor.py
			- trajector.py
			- __pycache__
	    - resource/
		- donnees.csv
		- scripts.sh
            - test/
            	- config.py
            	- crawler.py
            	- motor.py
            	- test_copyright.py
            	- test_crawler_motor.py
            	- test_flake8.py
            	- test_imu.py
            	- test_instructions_dir.py
            	- test_instructions_speed.py
            	- test_pep257.py
            	- __pycache__
            - package.xml
            - setup.cfg
            - setup.py


All the code used to run the crawler is written in Python and uses ROS2.
Here's a short explanation of each code in the repertory /crawler. 

calibrate.py : code used to calculate the coordinates of the crawler's direction in real time.

calibrator.py : code used to calibrate the crawler.

config.py : code used to configure odroid pins (PWM, GPIO...)

control_dir.py : code used to control the direction of the crawler (listen to the steering instructions published on the /instruction topic).

control_speed.py : code used to control crawler speed (listen to the speed instructions published on the /speed topic).

crawler.py : same code as for crawler.py in the /test directory, it defines the crawler class.

motor.py : same code as for motor.py in the /test directory, it defines the motor class.

trajector.py : code used to control the crawler using a setpoint coordinate (also published on the /instruction topic)

These codes, called nodes, communicate with each other via topics in which the codes publish messages and listen to them.
Here is the rqt graph representing the link between nodes.


	
This is the graph as it would look if all nodes were running at the same time.
What's more, not all nodes are working properly yet, and some need to be reworked. 
These include /trajector, which doesn't work yet but has already been sketched out, /steering_control, which needs to be written up, and /speed_instructions, which doesn't exist yet but works in the same way as /test_instructions_speed in /test repertory.

As for the /resource directory, it contains the no-python files used by some nodes to perform certain tasks. Here's an explanation of these files:

donnees.csv : Csv file used by calibrator.py to collect calibration data. /calibrator will then scan the data in this file to find the maximum and minimum magnetic field along the X and Y axes of the IMU. This will give us the North direction, which we can communicate to the /calibrate node.

scripts.sh : This file contains all the scripts used by motor.py to write to directories related to crawler control (PWM, GPIO...).
	 
Finally, the /test directory contains all the test codes to be used to check that the 
crawler is working properly.
Some of these codes are imperative to the robot's operation and must be executed 
before launching nodes in the /crawler directory.

test_imu.py : This code is used to start the IMU sensor, and must be executed first.  
Before executing this code, go to /dev, then execute the following line.
- sudo chmod 666 i2c-1

test_crawler_motor.py :  This code is used to start the crawler and test its movements. This code must be launched in a second step. 

test_instructions_dir : This code is used to test direction control by sending direction commands to /control_dir (/control_dir must be run in parallel).

test_instructions_speed : This code is used to test speed control by sending speed setpoints to /control_speed (/control_speed must be run in parallel).


## System installation and wiring

### Install the imu 

The Sparkfun Qwiic 9DoF ICM20948 IMU sensor module integrates a 3-axis gyroscope, a 3-axis accelerometer and a 3-axis magnetometer.
For more information on the sensor's features, you can visit the official website.
(https://www.sparkfun.com/products/15335)

The sensor is connected to the odroid c4 via 4 connectors.
VIN must be connected to the odroid c4's 3.3V DC power supply.
GND must be connected to a ground on the odroid c4.
DA (data) must be connected to pin i2c SDA while CL (clock) must be connected to pin i2c SCL.





When we set up the system we chose to connect the pins in this way:
VIN - pin 17
GND - pin 25
DA - pin 27
CL - pin 28

The diagram shows : 



### Install all components 

The rest of the connectors are listed below:

- pin 6 connected to GND of speed controller
- pin 13 connected to DIR input of left motor speed controller
- pin 16 connected to DIR input of right motor speed controller
- pin 33 connected to Unsoll input of right motor speed controller
- pin 35 connected to Unsoll input of left motor speed controller

You can find the details of these connections in the config.py code.

## Calibration

The calibration nodes are /calibrator who set up the compass for the crawler environment and /calibrate who measure the orientation of the crawler in real time. 
The calibrator node works by turning the crawler around himself during 40 seconds,  during which the crawler will take the magnetometer measures of the environment. 
At the end of the 40 seconds, the crawler will have the value of maximum and minimum for the 2 axes X and Y.  What will give us two functions of this type : 
					(schema)

Once we have these two functions, we can measure precisely the orientation of the crawler. 
The only problem is that we have 2 orientations for 1 value of magnetometer for the X axis due to the symmetry of the values along the axis.
So, we have to choose which value orientation is the right one. In order to do so, we use the second function for the Y axis. That’s what /calibrate node does. 
In this node, we convert the value of the X axis magnetometer to a degree value.   
