/*
 * @Author: your name
 * @Date: 2022-01-10 10:20:53
 * @LastEditTime: 2022-01-10 10:43:37
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/mapserver.cpp
 */
#include <iostream>
#include <ros/ros.h>
#include "graph_tool/SetPath.h"
#include "graph_tool/PathPoint.h"
#include "graph_tool/SetPoint.h"
#include <std_msgs/Int8.h>

class MapServer
{
private:
    ros::NodeHandle n;
    ros::Publisher pathpoint_pub;
    graph_tool::PathPoint pathpoint_a_b;
    ros::Publisher delete_point_pub;
    std_msgs::Int8 delete_point;
    ros::Publisher delete_path_pub;
    graph_tool::PathPoint delete_path_a_b;

public:
    MapServer(ros::NodeHandle &nh);
    ~MapServer();
    bool addpath_callback(graph_tool::SetPath::Request &req, graph_tool::SetPath::Response &res);
    bool deletepath_callback(graph_tool::SetPath::Request &req, graph_tool::SetPath::Response &res);
    bool deletepoint_callback(graph_tool::SetPoint::Request &req, graph_tool::SetPoint::Response &res);
};

MapServer::MapServer(ros::NodeHandle &nh):n(nh)
{
    pathpoint_pub = n.advertise<graph_tool::PathPoint>("/point_relationship", 10);
    delete_path_pub = n.advertise<graph_tool::PathPoint>("/delete_path", 10);
    delete_point_pub = n.advertise<std_msgs::Int8>("/delete_point", 10);
}

MapServer::~MapServer()
{
}

bool MapServer::addpath_callback(graph_tool::SetPath::Request &req, graph_tool::SetPath::Response &res)
{
    int source_ = req.sourcepoint;
    int target_ = req.targetpoint;
    if(source_ < 0 || target_ < 0 )
    {
        res.srvrecall = false;
        return false;
    }else{
        pathpoint_a_b.path_source = req.sourcepoint;
        pathpoint_a_b.path_target = req.targetpoint;
        pathpoint_pub.publish(pathpoint_a_b);
        res.srvrecall = true;
        return true;
    }
}

bool MapServer::deletepoint_callback(graph_tool::SetPoint::Request &req, graph_tool::SetPoint::Response &res)
{
    int pos = req.point;
    if(pos < 0)
    {
        res.recall = false;
        return false;
    }else{
        delete_point.data = req.point;
        delete_point_pub.publish(delete_point);
        res.recall = true;
        return true;
    }
}

bool MapServer::deletepath_callback(graph_tool::SetPath::Request &req, graph_tool::SetPath::Response &res)
{
    int source_ = req.sourcepoint;
    int target_ = req.targetpoint;
    if(source_ < 0 || target_ < 0 )
    {
        res.srvrecall = false;
        return false;
    }else{
        delete_path_a_b.path_source = req.sourcepoint;
        delete_path_a_b.path_target = req.targetpoint;
        delete_path_pub.publish(delete_path_a_b);
        res.srvrecall = true;
        return true;
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh;
    MapServer server(nh);

    ros::ServiceServer addpath_server = nh.advertiseService("line_two_points", &MapServer::addpath_callback, &server);
    ros::ServiceServer deletepath_server = nh.advertiseService("deletepath", &MapServer::deletepath_callback, &server);
    ros::ServiceServer deletepoint_server = nh.advertiseService("deletepoint", &MapServer::deletepoint_callback, &server);
    
    ros::spin();
    return 0;
}