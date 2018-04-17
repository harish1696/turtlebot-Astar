# Turtlebot Navigation - A* Path Planning
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
The module is capable of reading the velocity from the csv files and publishing it to the turtlebot in gazebo. The velocities in the .csv files are obtained using A* path planning.
![screenshot from 2018-04-17 09-41-25](https://user-images.githubusercontent.com/13302860/38873361-935a8fc4-4223-11e8-9068-e4cfa93324d7.png)

## Installing Dependencies
This program works on a device running Ubuntu 16.04 and ROS Kinetic Kame.

To install ROS Kinetic Kame in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

## How to build
Open a terminal window and run the following commands

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/harish1696/turtlebot-Astar
cd ..
catkin_make
```
## How to run Simulation
Open a terminal window and run the following commands

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch turtlebot_spawn turtlebot_gmap.launch
```
This launches the gazebo environment. Meanwhile, the program expects the user to input the full path name to the directory the velocity .csv files. Once entered, the program reads the .csv file and publishes them to the turtlebot.

## Known Issues
There is always a little drift in the path traversed by the turtlebot. 
