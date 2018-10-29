# ROS Beginner Tutorials


## Overview
The repo shows the simple publisher/subscriber examples from ROS wiki.  
A talker/listener node is demonstrated in the repo to publish and subscribe a custom string message.


## Dependencies
- Ubuntu Xenial (16.04)  
- ROS Kinetic  

## How to build
- Make sure you have a catkin workspace by this [link](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).  
- Open a new terminal, and execute the following command.  
```
cd <catkin workspace>
source devel/setup.bash
cd src
git clone https://github.com/CP-TSAI/beginner_tutorials.git
cd ..
catkin_make --only-pkg-with-deps beginner_tutorials
```




## How to run

- cd <catkin workspace>

- Use 3 terminals and execute the following commands. 

- terminal1

```
source devel/setup.bash
roscore
```

- terminal2

```
source devel/setup.bash
rosrun beginner_tutorials talker
```

- terminal3

```
source devel/setup.bash
rosrun beginner_tutorials listener
```






## Reference
- http://wiki.ros.org/ROS/Tutorials/NavigatingTheWiki

- http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem 

- http://wiki.ros.org/ROS/Tutorials/CreatingPackage

- http://wiki.ros.org/ROS/Tutorials/BuildingPackages 

- http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes 

- http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics 

- http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 

- http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber


