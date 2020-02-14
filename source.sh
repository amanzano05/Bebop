#!/bin/bash  

#This is a very special script that was handcrafted by yours truly, Brandon Hernandez :)


echo  "Launching the bebop driver..."
	sleep 0.5

xterm -e roslaunch bebop_driver bebop_node.launch & 
 

echo "Ready!!!"
	sleep 1

