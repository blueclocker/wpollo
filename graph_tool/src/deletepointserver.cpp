/*
 * @Author: your name
 * @Date: 2022-01-03 19:18:24
 * @LastEditTime: 2022-01-03 19:57:31
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/deletepointserver.cpp
 */
#include <iostream>
#include <ros/ros.h>
#include "graph_tool/SetPoint.h"
#include <std_msgs/Int8.h>

ros::Publisher deletepointpub;
static std_msgs::Int8 node;

bool deletepoint_callback(graph_tool::SetPoint::Request &req, graph_tool::SetPoint::Response &res)
{
    int pos = req.point;
    if(pos < 0)
    {
        res.recall = false;
        return false;
    }else{
        node.data = req.point;
        deletepointpub.publish(node);
        res.recall = true;
        return true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointdelete_server");
    ros::NodeHandle nh;
    deletepointpub = nh.advertise<std_msgs::Int8>("/delete_point", 10);

    ros::ServiceServer server = nh.advertiseService("deletepoint", deletepoint_callback);
    ros::spin();
    return 0;
}