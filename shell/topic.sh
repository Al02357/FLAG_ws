#! /usr/bin/env bash

RUN_AFTER_BASHRC="rostopic echo astar_node/grid_path" gnome-terminal --title="topic" & 
sleep 1
RUN_AFTER_BASHRC="rostopic echo " gnome-terminal --title="waypoint_pub" --tab & 
sleep 1
wait
exit 0
