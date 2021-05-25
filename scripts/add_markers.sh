#!/bin/sh
xterm -e " roslaunch my_robot world.launch" &
sleep 30
xterm -e " roslaunch my_robot amcl.launch" &
sleep 10
xterm -e " rosrun rviz rviz -d $(rospack find my_robot)/rviz/newRvizConfig.rviz" &
sleep 10
xterm -e " rosrun add_markers add_markers" &
sleep 5
xterm -e " rosparam set pick_up_location_reached true" &
sleep 5
xterm -e " rosparam set drop_off_location_reached true"

