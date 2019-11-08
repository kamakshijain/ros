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
cd ~/catkin_ws/
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
cd ~/catkin_ws/
catkin_make
```

## Publisher/subscriber working demo

Open a new terminal and give the following commands
```
roscore
```

Open a new terminal and give the following commands
```
source /devel/setup.bash
rosrun beginner_tutorials talker
```

Open a new terminal and give the following commands
```
source /devel/setup.bash
rosrun beginner_tutorials listener
```
## To run using launch file

To run the talker and listener nodes using launch file, follow the given steps.
```
roscore
```

Open a new terminal and give the following commands
```
source /devel/setup.bash
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
source /devel/setup.bash
rosservice call /change_string "Hello"
```
This will update the base string published by the talker to "Hello"

## Checking the log messages
The output of rqt_console with info and warn logger level messages has been added in the result directory of the repository. To run the GUI for checking logs run
```
rqt_console
```

## Running ROSTEST

Unit test cases for the project have been written using gtest and rostest. To run the tests using catkin go to to catkin workspace root directory and issue the following command:
```
catkin_make run_tests_beginner_tutorials
```

This will run the test suite and outout the result on the terminal as follows:
```
[Testcase: testtestTalker] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-testTalker/testServiceExistence][passed]
[beginner_tutorials.rosunit-testTalker/testServiceRun][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/kamakshi/.ros/log/rostest-kamakshi-ZenBook-Pro-15-UX550GE-UX550GE-4283.log

```


## Inspecting TF frames
The talker node broadcasts the /talk child frame with static translation and rotation with /world parent frame. To inspect the tf frames, first run the talker and listener nodes using roslaunch as follows:
```
roslaunch beginner_tutorials beginner_tutorial.launch
```
Now in new terminal, run the tf_echo command to verify the tf frames between talk and world as below:
```
rosrun tf tf_echo /world /talk
```
The above command will give output similar to as shown below:
```
At time 1573185893.564
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
            in RPY (radian) [3.140, 1.570, 2.000]
            in RPY (degree) [179.909, 89.954, 114.592]
At time 1573185894.264
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
	    in RPY (radian) [3.140, 1.570, 2.000]
	    in RPY (degree) [179.909, 89.954, 114.592]
At time 1573185895.264
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
            in RPY (radian) [3.140, 1.570, 2.000]
            in RPY (degree) [179.909, 89.954, 114.592]
At time 1573185896.264
```

To view the visualization of tf frames broadcasted from reference frame to other frame, we can use rqt_tf_tree command as shown below:
```
rosrun rqt_tf_tree rqt_tf_tree
```

## Recording bag files
To launch thr nodes and enable recording of all topics published by the nodes, use the given command:
```
roslaunch beginner_tutorials beginner_tutorial.launch record:=true
```
This command will record the data published on the /chatter topic by node /talker and create a listener.bag file in result directory.

## Examining and playing the recorded bag file
To examine the recorded rosbag file, run the following command:
```
rosbag info result/listener.bag
```

It will output the given info
```
path:        result/listener.bag
version:     2.0
duration:    28.3s
start:       Nov 08 2019 14:49:28.55 (1573242568.55)
end:         Nov 08 2019 14:49:56.84 (1573242596.84)
size:        341.3 KB
messages:    1688
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      281 msgs    : std_msgs/String
             /rosout       565 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   561 msgs    : rosgraph_msgs/Log 
             /tf           281 msgs    : tf2_msgs/TFMessage
```



