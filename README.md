# ENPM 662: Final Project
### Authors:
**Shantanu Parab**  |  sparab@umd.edu  
**Vineet Singh**  | vsingh03@umd.edu  

# The details of project approach and methodology used can be seen from the Project report.

# Kuka Robot
***
Downloading Dependencies
```bash
sudo  apt-get  install  ros-noetic-ros-control ros-noetic-ros-controllers
```
***
### Gazebo Simulation
Run the following commands to start the simulation
```bash
roslaunch kuka_robot gazebo.launch
#In another terminal
rosrun kuka_robot arm_control.py

```
***
### Package Installation
***
Zip Method  

1. Download Zip  
2. Extract it in your ROS workspace src folder  
3. Build the package  
***
Git Clone

```bash
cd catkin_ws/src
git clone 
cd ..
catkin_make
```
***
### Note
 The arm_control.py takes a initial setup time to compute the trajectory and joint angles using inverse kinematics.  
 The robot will start moving once this initial time is over.
