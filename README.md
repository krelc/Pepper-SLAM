SLAM on Pepper Robot with ROS 2
This repository contains the project and package for using the Pepper Robot with SLAM, utilizing the SLAMTEC RPLidar A1M8 sensor. This setup allows the Pepper Robot to perform SLAM within the ROS 2 environment.

Requirements
This package requires additional repositories:

naoqi2_driver: A ROS 2 driver for Pepper, available here.
SLAMTEC ROS2 driver: The ROS 2 driver for the SLAMTEC RPLidar A1M8, available here.
Installation Steps
1. Install naoqi2_driver
Follow the instructions in the naoqi2_driver repository to install the driver.

2. Configure the Pepper Robot
SSH into Pepper: Connect to the Pepper robot via SSH and disable autonomous movement:
bash
Copy code
ssh nao@<robot_ip>
Disable Autonomous Movement:
bash
Copy code
qicli call ALAutonomousLife.setState disabled
Wake Up Pepper:
bash
Copy code
qicli call ALMotion.wakeUp
3. Launch the naoqi Driver
Run the driver with the following command:

bash
Copy code
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_ip> qi_listen_url:=tcp://0.0.0.0:0
Note: After launching, use ros2 topic list to verify that the topics are active and ready for controlling the robot.

4. Install and Configure the SLAMTEC RPLidar A1M8
Clone and install the SLAMTEC ROS2 driver.
Ensure the RPLidar is enabled and functioning properly.
Final Setup and Running the Project
Build the Repository: Once all dependencies are installed and configured, build this repository using the following command:
bash
Copy code
colcon build --packages-select <your_package_name>
Attach the RPLidar: Print the provided mount using the 3mf file and securely attach the A1M8 sensor to the Pepper robot.
Usage
Once the setup is complete, you can start SLAM by launching the appropriate ROS 2 nodes. This will allow the Pepper robot to navigate using SLAM.
