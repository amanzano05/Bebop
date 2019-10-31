#!/bin/bash
#This is a very special script that was handcrafted by yours truly, Brandon Hernandez :)



echo -e "Sourcing ros kinetic into bash..."
        sleep 1

source /opt/ros/kinetic/setup.bash

source ~/bebop_ws/devel/setup.bash
echo -e "Sourced!"

echo -e "Launching the bebop driver..."

xterm -e roslaunch bebop_driver bebop_node.launch & 

	sleep 1 

echo -e "Sourced!"


