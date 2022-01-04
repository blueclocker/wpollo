/*
 * @Author: your name
 * @Date: 2022-01-03 19:48:14
 * @LastEditTime: 2022-01-03 19:55:56
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/deletelineserver.cpp
 */
#include <iostream>
#include <ros/ros.h>
#include "graph_tool/SetPath.h"
#include "graph_tool/PathPoint.h"

ros::Publisher deletepathpub;
static graph_tool::PathPoint path_a_b;

bool deletepath_callback(graph_tool::SetPath::Request &req, graph_tool::SetPath::Response &res)
{
    int source_ = req.sourcepoint;
    int target_ = req.targetpoint;
    if(source_ < 0 || target_ < 0 )
    {
        res.srvrecall = false;
        return false;
    }else{
        path_a_b.path_source = req.sourcepoint;
        path_a_b.path_target = req.targetpoint;
        deletepathpub.publish(path_a_b);
        res.srvrecall = true;
        return true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deletepath_server");
    ros::NodeHandle nh;
    deletepathpub = nh.advertise<graph_tool::PathPoint>("/delete_path", 10);

    ros::ServiceServer server = nh.advertiseService("deletepath", deletepath_callback);
    ros::spin();
    return 0;
}