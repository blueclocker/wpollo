/*
 * @Author: your name
 * @Date: 2022-01-03 19:18:11
 * @LastEditTime: 2022-01-03 19:57:24
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/deletepointclient.cpp
 */
#include <iostream>
#include <ros/ros.h>
#include "graph_tool/SetPoint.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deletepoint_client");
    ros::NodeHandle nh;
    if(argc != 2)
    {
	   ROS_INFO("usage: set point to be delete X" );
	   return 1;
    }
    ros::ServiceClient client = nh.serviceClient<graph_tool::SetPoint>("deletepoint");
    graph_tool::SetPoint srv;
    srv.request.point = std::atoi(argv[1]);

    if(client.call(srv))
    {
        ROS_INFO("delete point successful");
    }else{
        ROS_ERROR("Failed to call service");
    }

    return 0;
}