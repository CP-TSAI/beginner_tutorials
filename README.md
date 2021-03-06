# ROS Beginner Tutorials


## Overview
- The repo shows publisher/subscriber examples from ROS wiki.  
- A talker/listener node is demonstrated to publish and subscribe a custom string message.
- A service is added to change the published message.


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
git clone -b Week10_HW https://github.com/CP-TSAI/beginner_tutorials.git
cd ..
catkin_make --only-pkg-with-deps beginner_tutorials
```


## Setup ROS Environment with Terminal
- Before executing any ROS program
```
$ roscore
```

- Everytime when open a new terminal
```
$ source ~/catkin_ws/devel/setup.bash
```



## How to run

Use either **(1) roslaunch** or **(2) rosrun** command.


(1) **roslaunch**

```
- $ roscd beginner_tutorials

- $ roslaunch beginner_tutorials all.launch freq:=1
```

- The default publisher frequency is 10, you can change it by the roslaunch command.  


(2) **rosrun**

- cd to catkin_ws

- Use 2 terminals and execute the following commands. 

- terminal1

```
$ rosrun beginner_tutorials talker
```

- terminal2

```
$ rosrun beginner_tutorials listener
```


## How to use the service call

- Once the publisher and subscriber is running, we can use a **service call** to change the string message. 

- Open a new terminal, then run

```
$ rosservice call /change_string DESIRED_MESSAGE
``` 


## License
- Mit License
```
Copyright <2018> <Chin-Po Tsai>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions
of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
```





## Example Showcase

- make the frequency of the publisher 5hz.

```
$ roslaunch beginner_tutorials all.launch freq:=5
```

- You'll see something like
```
[ INFO] [1541551721.920444997]: I heard: [wakanda forever  85]
[ INFO] [1541551722.119970793]: wakanda forever  86
[ INFO] [1541551722.120464329]: I heard: [wakanda forever  86]
[ INFO] [1541551722.319971642]: wakanda forever  87
[ INFO] [1541551722.320449526]: I heard: [wakanda forever  87]
[ INFO] [1541551722.519969315]: wakanda forever  88
[ INFO] [1541551722.520364922]: I heard: [wakanda forever  88]
```

- If you want to change the published message, then do

```
$ rosservice call /change_string "wakanda is doomed"
```	

- The terminal would shows something like ...

```
[ INFO] [1541551856.720157134]: I heard: [wakanda forever  759]
[ INFO] [1541551856.919995812]: wakanda forever  760
[ WARN] [1541551856.920119453]: Changing the output String
[ INFO] [1541551856.920420078]: I heard: [wakanda forever  760]
[ INFO] [1541551857.119965528]: wakanda is doomed 761
[ INFO] [1541551857.120448497]: I heard: [wakanda is doomed 761]
[ INFO] [1541551857.319967544]: wakanda is doomed 762
[ INFO] [1541551857.320436805]: I heard: [wakanda is doomed 762]
```


## Reference
- http://wiki.ros.org/ROS/Tutorials
