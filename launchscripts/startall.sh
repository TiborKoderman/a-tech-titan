#!/bin/bash

cd ..

gnome-terminal -- roslaunch exercise3 rins_world.launch &

sleep 5

gnome-terminal -- roslaunch exercise3 amcl_simulation.launch &

gnome-terminal -- roslaunch turtlebot_rviz_launchers view_navigation.launch &