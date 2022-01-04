/*
 * @Author: your name
 * @Date: 2022-01-03 19:48:28
 * @LastEditTime: 2022-01-03 19:51:20
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/deletelineclient.cpp
 */
#include <iostream>
#include <ros/ros.h>
#include "graph_tool/SetPath.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deletepath_client");
    ros::NodeHandle nh;
    if(argc != 3)
    {
	   ROS_INFO("usage: set path to be delete X Y" );
	   return 1;
    }
    ros::ServiceClient client = nh.serviceClient<graph_tool::SetPath>("deletepath");
    graph_tool::SetPath srv;
    srv.request.sourcepoint = std::atoi(argv[1]);
    srv.request.targetpoint = std::atoi(argv[2]);

    if(client.call(srv))
    {
        ROS_INFO("delete path successful");
    }else{
        ROS_ERROR("Failed to call service");
    }

    return 0;
}