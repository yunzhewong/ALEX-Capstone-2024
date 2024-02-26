ALEX Capstone 2024 Project

ROS Simulation built with ROS2 Humble and Gazebo

I run it using the Windows Subsystem for Linux (WSL)

As of 24/02, launch gazebo sim by doing the following:
Enter the launch folder and run > ros2 launch gazebo.launch.py

As of 26/02: run 
in ~/ALEX-Capstone-2024

colcon build
ros2 run alex_nodes imu_subscriber
ros2 run alex_nodes torque_publisher

to simulate some motor torques and imu plotting

There is a camera topic as well, add a camera to RVIZ to visualise

This launches the robot model in Gazebo for simulation of the joints

Useful Links: 
https://docs.ros.org/en/humble/index.html (ROS2 Humble Documentation)

https://www.youtube.com/watch?v=laWn7_cj434&t=619s&ab_channel=ArticulatedRobotics (Gazebo Simulation Setup)

