source devel/setup.bash 
#! /bin/bash
sudo chmod 777 /dev/ttyTHS0 << EOF
nvidia
EOF
roslaunch fsm UAV1_single.launch
