#!/bin/sh
xterm -e " roslaunch my_robot world.launch" &
sleep 30
xterm -e " roslaunch my_robot amcl.launch" &
sleep 10
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch"
