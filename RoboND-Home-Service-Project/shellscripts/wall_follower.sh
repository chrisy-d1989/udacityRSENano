#!/bin/sh
xterm  -e  "cd catkin_ws; source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/world/world.world" &
sleep 5
xterm  -e  "cd catkin_ws; source /home/workspace/catkin_ws/devel/setup.bash;roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm  -e  "cd catkin_ws; source /home/workspace/catkin_ws/devel/setup.bash; roslaunch wall_follower wall_follower.launch"