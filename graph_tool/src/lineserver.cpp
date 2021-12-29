/*
 * @Author: your name
 * @Date: 2021-12-28 22:19:17
 * @LastEditTime: 2021-12-28 22:38:11
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/lineserver.cpp
 */
#include <iostream>
#include <ros/ros.h>
#include "graph_tool/SetPath.h"
#include "graph_tool/PathPoint.h"

ros::Publisher pathpointpub;
static graph_tool::PathPoint node_a_b;

bool path_callback(graph_tool::SetPath::Request &req, graph_tool::SetPath::Response &res)
{
    int source_ = req.sourcepoint;
    int target_ = req.targetpoint;
    if(source_ < 0 || target_ < 0 )
    {
        res.srvrecall = false;
        return false;
    }else{
        node_a_b.path_source = req.sourcepoint;
        node_a_b.path_target = req.targetpoint;
        pathpointpub.publish(node_a_b);
        res.srvrecall = true;
        return true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_server");
    ros::NodeHandle nh;
    pathpointpub = nh.advertise<graph_tool::PathPoint>("/point_relationship", 10);

    ros::ServiceServer server = nh.advertiseService("line_two_points", path_callback);
    ros::spin();
    return 0;
}