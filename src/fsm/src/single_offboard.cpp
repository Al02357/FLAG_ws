/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/TwistStamped.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
#include <eigen3/Eigen/Dense>

#include <fsm/command_acc.h>
using namespace std;

static mavros_msgs::State current_state;
static int flight_mode=1;
static int waypoint_num=0;
static int j=1;
static int cmdd=0;
static int flag1=0;
static int flag2=0;
static int ready_fly=0;
static float voltage_now;
static double waypoint[50][3];
static geometry_msgs::PoseStamped aim;
Eigen::Vector3d pos_drone_fcu;
//pos_drone_fcu = Eigen::Vector3d(0.0,0.0,0.0);

void vtg_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    voltage_now = msg->voltage;
}

void UdpListen(const uint16_t cport)
{
    ros::NodeHandle nh;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        ROS_ERROR("Network Error");
        return;
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_lis;
    int len;
    memset(&addr_lis, 0, sizeof(struct sockaddr_in));
    addr_lis.sin_family = AF_INET;
    addr_lis.sin_port = htons(cport);
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_lis.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
    len = sizeof(addr_lis);

    /* 绑定socket */
    if(bind(sock_fd, (struct sockaddr *)&addr_lis, sizeof(addr_lis)) < 0)
    {
      perror("bind error:");
      exit(1);
    }


    int recv_num;
    char recv_buf[100];
    const char dot[2] = ",";
    struct sockaddr_in addr_client;

    while(ros::ok()){
        char *p;
        int ent=0;
        int cmd;
        double px,py,pz;

        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);

        if(recv_num < 0||abs(recv_num-19)>3)
        {
            ROS_ERROR("Recv Fail!");
            continue;
        }
        recv_buf[recv_num] = '\0';
//      ROS_INFO("Rec: %s, len=%d",recv_buf,recv_num);

        p = strtok(recv_buf,dot);
        sscanf(p,"%d",&cmd);
        p=strtok(NULL,dot);
        sscanf(p,"%lf",&px);
        p=strtok(NULL,dot);
        sscanf(p,"%lf",&py);
        p=strtok(NULL,dot);
        sscanf(p,"%lf",&pz);
        if(cmd!=6)
        {
          aim.pose.position.x=px;
          aim.pose.position.y=py;
          aim.pose.position.z=pz;
        }
        ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf",cmd,aim.pose.position.x,aim.pose.position.y,aim.pose.position.z);
        if(flag2==0)
        cmdd=cmd;
         switch (cmdd)
        {
        case 0:
            if(ready_fly==1)
            cout<<"Ready Fly"<<endl;
            else
            ROS_INFO("Waiting");
            break;
            
        case 1:
            if(pos_drone_fcu[2]>1.5||voltage_now<13.5)
            {
            cmdd=4;
            flag2=1;
            }
            break;
        case 9:
            if(pos_drone_fcu[2]>1.5||voltage_now<13.5)
            {
            cmdd=4;
            flag2=1;
            }
            break;
            
        }
        //   cout<<voltage_now<<endl;
        //    cout<<pos_drone_fcu[2]<<endl;
    }

}

void ready_cb(const fsm::command_acc &msg)
{ 
    ready_fly=msg.ready;
    //ROS_ERROR("11111111");voltage_now
    //cout<<"1111";
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_fcu  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>   
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber ready= nh.subscribe("/px4/ready",10,ready_cb);
    ros::Subscriber voltage_sub = nh.subscribe<sensor_msgs::BatteryState>
            ("/mavros/battery", 300, vtg_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>  
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>   
            ("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>    
            ("mavros/setpoint_position/local", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    //parameters
    nh.param("/single_offboard/way_num",waypoint_num,0);
    aim.pose.position.x = 0;
    aim.pose.position.y = 0;
    aim.pose.position.z = 0.5;
    //point
    for (int i = 1; i <= waypoint_num; i++)
    {
      nh.param("/single_offboard/waypoint" + to_string(i) + "_x", waypoint[i][0], -1.0);
      nh.param("/single_offboard/waypoint" + to_string(i) + "_y", waypoint[i][1], -1.0);
      nh.param("/single_offboard/waypoint" + to_string(i) + "_z", waypoint[i][2], -1.0);
    }
//    nh.param("/single_offboard/waypoint1_x",aim.pose.position.x,1.0);
//    nh.param("/single_offboard/waypoint1_y",aim.pose.position.y,2.0);
//    nh.param("/single_offboard/waypoint1_z",aim.pose.position.z,3.0);
    new std::thread(&UdpListen,12001);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    sensor_msgs::BatteryState voltage_now;
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode="OFFBOARD";

    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "LAND";

    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd,disarm_cmd;
    arm_cmd.request.value = true;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(aim);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        //waypoint
        if(cmdd==6)
        {   
//          if(fabs(pos_drone_fcu[0] - waypoint[j][0]) < (0.05) && fabs(pos_drone_fcu[1] - waypoint[j][1]) < (0.05) && fabs(pos_drone_fcu[2] - waypoint[j][2]) < (0.05))
          
if(sqrt(pow(pos_drone_fcu[0] - waypoint[j][0],2)+pow(pos_drone_fcu[1] - waypoint[j][1],2)+pow(pos_drone_fcu[2] - waypoint[j][2],2))<(0.1) && j<waypoint_num)
          j=j+1;
          aim.pose.position.x = waypoint[j][0];
          aim.pose.position.y = waypoint[j][1];
          aim.pose.position.z = waypoint[j][2];
        }
        //fsm
        if(cmdd!=4)local_pos_pub.publish(aim);
        if(cmdd==1){
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_ERROR("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
        }
        if(cmdd==2){
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_ERROR("Offboard enabled");
                }
                last_request = ros::Time::now();
            }else if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_ERROR("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        if(cmdd==9)
        {
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                  ROS_ERROR("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            else if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
               if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_ERROR("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
                  ROS_ERROR("Vehicle armed");
        }
        if(cmdd==4)
        {   
        if(abs(pos_drone_fcu[2] - 0.1) < (0.05))
      { 
        flag1=1;
      }
        if(flag1==0)
      {
          aim.pose.position.x = pos_drone_fcu[0];
          aim.pose.position.y = pos_drone_fcu[1];      
          aim.pose.position.z = 0.1;
          local_pos_pub.publish(aim);
      }
        else
      {
        if(current_state.mode == "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "Hold";
                set_mode_client.call(mode_cmd);
                ROS_ERROR("1111");
            }

            if(current_state.armed)
            {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
                ROS_ERROR("2222");
            }

            if (arm_cmd.response.success)
            {
                ROS_ERROR("Disarm successfully!");
            }
      }
        }

        if(cmdd==5)
    {
            if(current_state.mode == "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "MANUAL";
                set_mode_client.call(mode_cmd);
            }

            if(current_state.armed)
            {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
            }

            if (arm_cmd.response.success)
            {
                ROS_ERROR("Disarm successfully!");
            }
    }
    cout<<current_state.mode<<endl;
        ros::spinOnce();
        rate.sleep();
   }
    return 0;
}

