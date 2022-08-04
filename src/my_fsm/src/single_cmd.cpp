// lib
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include "fsm_func.h"
// message


// namespace
using namespace std;
using namespace fsm_func;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"drone_cmd");
    ros::NodeHandle nh;
    drone_controller controller(nh);
    controller.run();
    return 0;
}