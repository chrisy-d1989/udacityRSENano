#!/bin/sh
xterm  -e  "cd catkin_ws; source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/world/world.world" &
sleep 5
xterm  -e  "cd catkin_ws; source /home/workspace/catkin_ws/devel/setup.bash;roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/world/my_map.yaml "&
sleep 5
xterm  -e  "cd catkin_ws; source /home/workspace/catkin_ws/devel/setup.bash; rosrun rviz rviz -d /home/workspace/catkin_ws/src/rvizConfig/config.rviz" &
sleep 5
xterm  -e  "cd catkin_ws; source /home/workspace/catkin_ws/devel/setup.bash; roslaunch add_markers add_markers.launch"&
sleep 5
xterm  -e  "cd catkin_ws; source /home/workspace/catkin_ws/devel/setup.bash; roslaunch pick_objects pick_objects.launch"