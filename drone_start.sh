#This is a very special script that was handcrafted by yours truly, Brandon Hernandez :)




echo -e "Sourcing the bebop workspace to the bash...."
	sleep 2 

source ~/bebop_ws/devel/setup.bash
echo -e "Sourced!"

echo -e "Launching the bebop driver..."
	sleep 2

xterm -e roslaunch bebop_driver bebop_node.launch & 

	sleep 8 

echo -e "Sourcing ros kinetic into bash..."
        sleep 2 

source /opt/ros/kinetic/setup.bash
echo -e "Sourced!"
echo -e "Running KeyCommands..."
	sleep 2 
python ~/Documents/Bebop/KeyCommands.py


