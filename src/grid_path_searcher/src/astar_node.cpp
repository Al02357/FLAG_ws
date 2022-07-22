/*
输入：
输出：
*/
#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "Astar_searcher_2d.h"
// #include "JPS_searcher.h"
//#include "backward.hpp"
#include <sensor_msgs/point_cloud_conversion.h>


using namespace std;
using namespace Eigen;

// namespace backward {
// backward::SignalHandling sh;
// }
/*----------------------global variables----------------------*/
// Simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size;    

bool _has_map   = false;
Vector2d _start_pt;
double actual_height = 0.0;
// Map size
Vector2d _map_lower, _map_upper;
int _max_x_id, _max_y_id;
/*----------------------ROS----------------------*/
// SubPub
ros::Subscriber _map_sub, _pts_sub,_nav_sub;//读地图与目标点
ros::Publisher  _grid_path_vis_pub, 
                               _visited_nodes_vis_pub,
                               _grid_map_vis_pub,
                               _grid_path_pub,
                               _grid_twist_pub;
// Callback
void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud::ConstPtr & pointcloud_map_raw);
void simPoseCallback(const geometry_msgs::PoseStamped & msg);
/*----------------------A*----------------------*/
AstarPathFinder2d * _astar_path_finder     = new AstarPathFinder2d();
void pathFinding(const Vector2d start_pt, const Vector2d target_pt);
/*----------------------visual----------------------*/
void visGridPath( vector<Vector2d> nodes, bool is_use_jps );                                            //可视化函数
void visVisitedNode( vector<Vector2d> nodes );           
/*----------------------message pub----------------------*/
void pubGridPath(vector<Vector2d> nodes);
void pubGridTwist(vector<Vector2d> nodes);

/*----------------------function----------------------*/
void rcvPointCloudCallBack(const sensor_msgs::PointCloud::ConstPtr & pointcloud_map_raw)
{   
    // ROS_WARN("?");
    // cout<<"START. . ";
    sensor_msgs::PointCloud2 pointcloud_map;
    convertPointCloudToPointCloud2(*pointcloud_map_raw, pointcloud_map);
    // if(_has_map){
    //     //QUES 是否需要zhiling？
    //     _astar_path_finder->cleanObs();
    // }
     
    pcl::PointCloud<pcl::PointXYZ> cloud;           //容器填充为三维点
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;
     
    pcl::fromROSMsg(pointcloud_map, cloud);//类型变换
     
    if( (int)cloud.points.size() == 0 ) return;//空地图则返回
     
    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)//遍历点云中的点
    {
        //FIXME 输入点云是三维的，这里需要确定z的范围
        //QUES 是不是0上下的一个范围？
        // if(高度不在范围内) continue;
        pt = cloud.points[idx];        //取点到pt
        //FIXME 修改0.5为目标高度范围
        if(std::abs(pt.z-actual_height)>_resolution/2) continue;
        // cout<<idx<<"::::"<<pt.z<<"::::"<<endl;
         _astar_path_finder->setObs(pt.x, pt.y);
        //  cout<<"ptx::::"<<pt.x<<"    pty::::"<<pt.y<<endl;
// cout<<". ";
        // for visualize only
        // 可视化
        Vector2d cor_round = _astar_path_finder->coordRounding(Vector2d(pt.x, pt.y));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = 0;
        cloud_vis.points.push_back(pt);
        // 可视化end
    }
     
    // 可视化
    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);  //将处理好的可视化数据发布到ROS话题中
 cout<<"pointcloud pub."<<endl;
    map_vis.header.frame_id = "t265_odom_frame";
    _grid_map_vis_pub.publish(map_vis);//demo_node/grid_map_vis
    // 可视化end

    _has_map = true;
    cout<<"pointcloud received."<<endl;
}

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    //USAGE 拿到waypoint消息后，检查并进行路径寻找
    //起点为全局变量_start_pt，终点为消息中的waypoint
    if( _has_map == false ) //无地图或点位于地下，退出函数
        return;

    Vector2d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                              wp.poses[0].pose.position.y;

    ROS_INFO("[node] receive the planning target");
    pathFinding(_start_pt, target_pt); 
}

void simPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    _start_pt(0) = msg.pose.position.x;
    _start_pt(1) = msg.pose.position.y;
    actual_height = msg.pose.position.z;
    // ROS_WARN("GET POSE");
}

void pathFinding(const Vector2d start_pt, const Vector2d target_pt)
{
    //Call A* to search for a path
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

    //Retrieve the path
    auto grid_path     = _astar_path_finder->getPath();
    auto grid_twist    = _astar_path_finder->getTwist2();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    //Visualize the result
    visGridPath (grid_path, false);
    visVisitedNode(visited_nodes);
    pubGridPath(grid_path);
    pubGridTwist(grid_twist);

    //Reset map for next call
    _astar_path_finder->resetUsedGrids();
}
/*----------------------main----------------------*/
int main(int argc, char** argv)
{
//init
    ros::init(argc, argv, "astar_node");
    ros::NodeHandle nh("~");

//sub
    _map_sub  = nh.subscribe( "map",       10, rcvPointCloudCallBack );
    _pts_sub     = nh.subscribe( "waypoints", 10, rcvWaypointsCallback );   
    _nav_sub    = nh.subscribe( "pose", 10, simPoseCallback );
//pub
    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
    _grid_path_pub                     = nh.advertise<nav_msgs::Path>("grid_path",1);
    _grid_twist_pub                     = nh.advertise<nav_msgs::Path>("grid_twist",1);
    

    nh.param("map/resolution",    _resolution,   0.2);
    nh.param("map/x_size"         ,        _x_size, 50.0);
    nh.param("map/y_size"         ,        _y_size, 50.0);    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);//QUES 是否每次从0开始运行？

    _map_lower  << - _x_size/2.0,  -  _y_size/2.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0;
    _inv_resolution = 1.0 / _resolution;
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);

    _astar_path_finder  = new AstarPathFinder2d();
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id);
    
//ROS
    ros::Rate rate(100);
    while(ros::ok()) 
    {
        // ROS_INFO("test");
        ros::spinOnce();      
        rate.sleep();
    }
    delete _astar_path_finder;
    return 0;
}

void pubGridPath(vector<Vector2d> nodes){
    nav_msgs::Path node_nav;
    node_nav.header.frame_id = "t265_odom_frame";
    geometry_msgs::PoseStamped pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.pose.position.x = coord(0);
        pt.pose.position.y = coord(1);
        pt.pose.position.z =  0;

        node_nav.poses.push_back(pt);
    }
    
    _grid_path_pub.publish(node_nav);
    
}
void pubGridTwist(vector<Vector2d> nodes){
    nav_msgs::Path node_nav;
    node_nav.header.frame_id = "t265_odom_frame";
    geometry_msgs::PoseStamped pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.pose.position.x = coord(0);
        pt.pose.position.y = coord(1);
        pt.pose.position.z =  0;

        node_nav.poses.push_back(pt);
    }
    
    _grid_twist_pub.publish(node_nav);
    
}


void visGridPath( vector<Vector2d> nodes, bool is_use_jps )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "t265_odom_frame";
    node_vis.header.stamp = ros::Time::now();
    
    if(is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if(is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = 0;

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode( vector<Vector2d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "t265_odom_frame";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = 0;

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}