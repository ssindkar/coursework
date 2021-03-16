#!/bin/sh
xterm -e " roslaunch my_robot world.launch world_file:=/home/workspace/catkin_ws/src/my_robot/worlds/Shalaka.world " &
sleep 10
xterm -e " roslaunch my_robot amcl.launch map_file:=/home/workspace/catkin_ws/src/map/map.yaml " &
sleep 5
xterm -e " roslaunch my_robot rviz_amcl.launch " &
sleep 5
xterm -e " rosrun pick_objects pick_objects " &
