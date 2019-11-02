# ROS Beginner Tutorials - Introduction to Publisher and Subscriber  
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview

This program is a basic introduction to a publisher and a subscriber in ROS. It creates two nodes, a talker (publisher) node and a listener (subscriber) node to demonstarte how they interact with each other.

## Dependencies
This program works on a device running Ubuntu 16.04 and ROS Kinetic.

To install ROS Kinetic in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

## Build Instructions

To run this code in a catkin workspace:
```
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash
cd src/
git clone https://github.com/kamakshijain/beginner_tutorials.git
cd beginner_tutorials
git checkout Week10_HW
catkin_make
```
If you do not have a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/kamakshijain/beginner_tutorials.git
cd ..
catkin_make
```

## Publisher/subscriber working demo

Open a new terminal and give the following commands
```
roscore
```

Open a new terminal and give the following commands
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker
```

Open a new terminal and give the following commands
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener
```
## To run using launch file

To run the talker and listener nodes using launch file, follow the given steps.
```
roscore
```

Open a new terminal and give the following commands
```
source ~/catkin_ws/devel/setup.bash
roslaunch beginner_tutorials beginner_tutorial.launch
```
You can specify the publisher frequency along with launch file as input argument to change the publisher frequency.
```
roslaunch beginner_tutorials beginner_tutorial.launch frequency:=20
```
This will start roscore and talker and listener nodes in two terminals. 

## Running the service

The change_string service has been added to the project which modifies the base string published by the talker.
After building the project using the build instructions above and launching the talker-listener nodes using roslaunch, we can give a demo for the /change_string service added for the talker node. To run the service, enter the following command:
```
source ~/catkin_ws/devel/setup.bash
rosservice call /change_string "Hello"
```
This will update the base string published by the talker to "Hello"

## Checking the log messages
The output of rqt_console with info and warn logger level messages has been added in the results directory of the repository. To run the GUI for checking logs run
```
rqt_console
```



