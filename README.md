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
source devel/setup.bash
cd src/
git clone https://github.com/kamakshijain/beginner_tutorials.git
cd ..
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
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker
```

Open a new terminal and give the following commands
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```

