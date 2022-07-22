catkin_make  
source devel/setup.bash

# apriltag 
## 安装 apriltag
git clone https://github.com/AprilRobotics/apriltag.git  
cd PATH_TO_APRILTAG  
mkdir build  
cd build   
make ..  
cd ..  

## 使用
roslaunch apriltag_ros continuous_detection.launch


# realsense_ros_gazebo
roslaunch realsense_ros_gazebo simulation_D435i_sdf.launch

# circles
python2 circle_recognition.py 
