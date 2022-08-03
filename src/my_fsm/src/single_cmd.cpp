// lib
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
// message
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
// namespace
using namespace std;

/*-----------------VARIABLE-----------------*/
mavros_msgs::State current_state;
Eigen::Vector3d pos_drone_fcu;
float voltage_now;
int cmd_code;

/*-----------------FUNCTION-----------------*/
void state_cb(const mavros_msgs::State::ConstPtr & msg)
{
    current_state = *msg;
    cout<<"[commander] Current state: "<<current_state.mode<<endl;
}
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_fcu  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}
void vtg_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    voltage_now = msg->voltage;
}
void joy_cb(const std_msgs::Int32::ConstPtr & msg)
{
    cmd_code = msg->data;
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"drone_cmd");
    ros::NodeHandle nh;
    ros::Subscriber joy_sub = nh.subscribe("/sim_joy/joy",10,joy_cb);
    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber voltage_sub = nh.subscribe("/mavros/battery", 300, vtg_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::Rate rate(20.0);
    /* 等待连接 */
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    /*  */
    geometry_msgs::PoseStamped* aim_for_offboard;
    aim_for_offboard->pose.position.x = 0.0;
    aim_for_offboard->pose.position.x = 0.0;
    aim_for_offboard->pose.position.x = 0.0;
    aim_for_offboard->pose.orientation.w = 1.0;
    aim_for_offboard->pose.orientation.x = 0.0;
    aim_for_offboard->pose.orientation.y = 0.0;
    aim_for_offboard->pose.orientation.z = 0.0;
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(*aim_for_offboard);
        ros::spinOnce();
        rate.sleep();
    }
    aim_for_offboard = NULL;


    while(ros::ok())
    {


        ros::spinOnce();
        rate.sleep();
    }

}