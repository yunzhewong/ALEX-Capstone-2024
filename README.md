ALEX Capstone 2024 Project

ROS Simulation built with ROS2 Humble and Gazebo

I run it using the Windows Subsystem for Linux (WSL)

As of 18/02/24, launch has two options
1. Enter the launch folder and run > ros2 launch model.launch.py

This launches the robot model in RViz for visualisation of the model

2. Or: Enter the launch folder and run > ros2 launch gazebo.launch.py

This launches the robot model in Gazebo for simulation of the joints

Useful Links: 
https://docs.ros.org/en/humble/index.html (ROS2 Humble Documentation)

https://www.youtube.com/watch?v=laWn7_cj434&t=619s&ab_channel=ArticulatedRobotics (Gazebo Simulation Setup)


For some reason I can't get the things to spawn in one launch file?

terminal 1: run "ros2 launch model.launch.py" // starts the robot-
terminal 2: run "ros2 launch gazebo_ros gazebo.launch.py"
terminal 3: run "ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity my_bot"
