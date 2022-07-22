source devel/setup.bash 
#!/bin/bash
sudo -S chmod 777 /dev/ttyTHS0 << EOF
nvidia
EOF
roslaunch fsm UAV3_single.launch

