Using ROS2 to craete a SLAM algorithm on a Pepper robot.
More will be added later.


This repository contains the project and package that enables the use of the Pepper Robot and SLAM, using the SLAMTEC RPLidar A1M8.
This package requires additional repositories, the first is the [naoqi2_driver](https://github.com/ros-naoqi/naoqi_driver2)
Firstly install the naoqi2 driver as per instructions. 
As per instructions: 
ssh into the Pepper and disable autonomous movement from the Pepper:
ssh nao@<robot_ip>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp

Run the driver:
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_ip> qi_listen_url:=tcp://0.0.0.0:0

When running the driver, using ros2 topic list should give a list of topics ready to control.
The SLAMTEC RPLidar A1M8 is used, with the driver repository [SLAMTEC ROS2](https://github.com/Slamtec/sllidar_ros2).
Ensure that the laser is enabled and can be run. 

Once all the setup is complete, build and run this repository. Print the mount using the 3mf file and attach the A1M8 to it.
