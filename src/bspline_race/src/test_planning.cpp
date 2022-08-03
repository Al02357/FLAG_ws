#include <ros/ros.h>
#include <bspline_race/bspline_race.h>

using namespace FLAG_Race;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "Test_planning");
    ros::NodeHandle nh("~");
    plan_manager manager(nh);
    ros::spin();
    return 0;
}

