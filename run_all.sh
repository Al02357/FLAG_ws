#! /usr/bin/env bash
set -e
cd ~/FLAG_ws
#catkin_make && sleep 1;
#RUN_AFTER_BASHRC="ls;ls --help" gnome-terminal --tab;
#RUN_AFTER_BASHRC="./QGroundControl.AppImage" gnome-terminal --title="QGC" --tab;

RUN_AFTER_BASHRC="roslaunch fsm iris_realsense_camera_px4_mavros_vo.launch" gnome-terminal --title="Gazebo Sim" --tab &
sleep 5 &&
RUN_AFTER_BASHRC="~/QGroundControl.AppImage" gnome-terminal --title="QGC" --tab;
RUN_AFTER_BASHRC="rosrun remap_node message_remap" gnome-terminal --title="remap" --tab &
sleep 2 &&
RUN_AFTER_BASHRC="roslaunch fsm UAV1_single.launch" gnome-terminal --title="connection" --tab;
RUN_AFTER_BASHRC="roslaunch fsm swarm.launch" gnome-terminal --title="control" --window & 
sleep 1 ;
RUN_AFTER_BASHRC="roslaunch mapping mapping.launch" gnome-terminal --title="fiesta" --tab ;
RUN_AFTER_BASHRC="roslaunch grid_path_searcher astar_node.launch" gnome-terminal --title="astar" --tab & 
sleep 1 &&
RUN_AFTER_BASHRC="roslaunch bspline_race traj_testing.launch" gnome-terminal --title="bspline" --tab &
sleep 1;


#RUN_AFTER_BASHRC="bash ~/FLAG_ws/shell/topic.sh" gnome-terminal --title="topic" --window & 
#sleep 1
wait
exit 0
