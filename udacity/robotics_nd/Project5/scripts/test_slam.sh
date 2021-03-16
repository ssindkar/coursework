#!/bin/sh
xterm -e " roslaunch my_robot world.launch world_file:=/home/workspace/catkin_ws/src/my_robot/worlds/Shalaka.world" &
sleep 10
xterm -e " roslaunch my_robot mapping.launch " &
sleep 5
xterm -e " rosrun teleop_twist_keyboard teleop_twist_keyboard.py " &
sleep 5
xterm -e " roslaunch my_robot rviz_mapping.launch " &
