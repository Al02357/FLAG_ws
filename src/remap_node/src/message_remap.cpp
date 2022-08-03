
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include<nav_msgs/Odometry.h>

char* frame_id;

ros::Subscriber _nav_sub;
ros::Publisher    _nav_pub;

void simPoseCallback(const nav_msgs::Odometry::ConstPtr & msg){
    nav_msgs::Odometry msg_;
    msg_ = *msg;
    msg_.header.frame_id = "t265_odom_frame";
    _nav_pub.publish(msg_);
}
int main(int argc,char** argv){

    ros::init(argc, argv, "remap_node");
    ros::NodeHandle nh("~");

    _nav_sub =  nh.subscribe( "/mavros/odometry/in", 10, simPoseCallback );
    _nav_pub = nh.advertise<nav_msgs::Odometry>("odom",1);
    
    ros::spin();

    return 0;
}