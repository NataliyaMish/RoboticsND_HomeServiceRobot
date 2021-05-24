# RoboticsND_HomeServiceRobot

This is repository for the fourth project of Udacity Robotics Software Engineer Nanodegree.

The goal of the project was to implement a home service robot that would pick-up and drop-off an object at known locations
using localization, mapping as well as navigation techniques.

It uses the following ROS packages:

* my_robot package - holds my robot and my world;
* [turtlebot_teleop package](http://wiki.ros.org/turtlebot_teleop) - provides teleoperation using joysticks or keyboard;
* [gmapping package](http://wiki.ros.org/gmapping) - provides laser-based SLAM (Simultaneous Localization and Mapping);
* [amcl package](http://wiki.ros.org/amcl) - implements the adaptive (or KLD-sampling) Monte Carlo localization approach (as described by Dieter Fox), 
  which uses a particle filter to track the pose of a robot against a known map;
* pick_objects package - uses ROS Navigation stack to guide robot to the pick-up/drop-off location
* add_markers package - simulates the object being picked-up/dropped-off

## How to test it?

#### Create a catkin_ws
```sh
$ cd /home/
$ mkdir -p /home/catkin_ws/src/
$ cd catkin_ws/src/
$ catkin_init_workspace
$ cd ..
```

#### Clone the repository to catkin_ws/src/
```sh
$ cd /home/catkin_ws/src/
$ git clone https://github.com/NataliyaMish/RoboticsND_HomeServiceRobot.git .
```

#### Build the packages
```sh
$ cd /home/catkin_ws/ 
$ catkin_make
```

#### Source your environment
```sh
$ cd /home/catkin_ws/
$ source devel/setup.bash
```

#### Give home_service.sh execute pemission
```sh
$ cd /home/catkin_ws/src/scripts
$ chmod +x home_service.sh
```

#### Run the home_service.sh script to see robot move
```sh
$ ./home_service.sh
```
