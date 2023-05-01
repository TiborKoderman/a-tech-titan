#!/bin/bash
# cd ..
gnome-terminal -- roslaunch exercise7 rins_world.launch

sleep 10

gnome-terminal -- roslaunch turtlebot_custom_navigation amcl_simulation.launch
gnome-terminal -- roslaunch turtlebot_rviz_launchers view_navigation.launch
gnome-terminal -- rosrun exercise6 detect_rings
gnome-terminal -- roslaunch exercise6 find_cylinder.launch