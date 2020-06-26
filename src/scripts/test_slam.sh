#!/bin/sh
# xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/map/my_world.world " &
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
# xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/world.world" &
#xterm -e " roslaunch my_robot world.launch " &
sleep 5

xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
#xterm -e " roslaunch my_robot gmapping.launch " &
sleep 5

xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch "
# xterm -e " roslaunch my_robot teleop.launch "