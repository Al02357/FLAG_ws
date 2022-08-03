#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <mavros_msgs/State.h>
#include <thread>

/*-----------------VARIABLE-----------------*/
int cmd_code;
bool ros_state = 1;
using namespace std;
/*-----------------FUNCTION-----------------*/
void get_input_cmd(ros::Rate rate)
{
    while(ros_state)
    {
        cin >> cmd_code;
        rate.sleep();
    }
}
int main(int argc,char** argv){
    ros::init(argc, argv, "sim_joy");
    ros::NodeHandle nh;

    ros::Rate rate(20.0);
    ros::Publisher joy_pub = nh.advertise<std_msgs::Int32>("/sim_joy/joy",1);
    // ros::Subscriber state_sub = nh.subscribe("state",1,state_callback);
    std_msgs::Int32 msg;
    msg.data = 0;

    thread t1(get_input_cmd,rate);
    ros_state = ros::ok();
    while(ros_state){
        cout<<"cmd_code"<<"\n";
        cout<<cmd_code<<endl;
        ros_state = ros::ok();
        msg.data = cmd_code;
        joy_pub.publish(msg);
        rate.sleep();
    }
    return 0;
}