#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
using namespace Eigen;
int main(int argc,char** argv)
{
    ros::init(argc,argv,"pub_target");
    ros::NodeHandle nh;

    ros::Publisher pub_ = nh.advertise<nav_msgs::Path>("/pub_target/target_list",1);

    nav_msgs::Path msg;
    geometry_msgs::PoseStamped pt;
    MatrixXd pt_v(2,2);
    pt_v<<0,-3,
                  4,-2;
    for(int i = 0;i<2;i++)
    {
        pt.pose.position.x = pt_v(i,0);
        pt.pose.position.y = pt_v(i,1);
        pt.header.seq = i+1;
        msg.poses.push_back(pt);
    }
    ros::Rate rate(1.0);
    while(ros::ok())
    {
        pub_.publish(msg);
        rate.sleep();
    }
}