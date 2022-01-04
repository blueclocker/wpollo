/*
 * @Author: your name
 * @Date: 2021-12-28 18:57:04
 * @LastEditTime: 2022-01-03 14:52:19
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/line.cpp
 */
#include <iostream>
#include <ros/ros.h>
#include "graph_tool/SetPath.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_client");
    ros::NodeHandle nh;
    if(argc != 3)
    {
	   ROS_INFO("usage: line_two_points_client X Y" );
	   return 1;
    }
    ros::ServiceClient client = nh.serviceClient<graph_tool::SetPath>("line_two_points");
    graph_tool::SetPath srv;
    srv.request.sourcepoint = std::atoi(argv[1]);
    srv.request.targetpoint = std::atoi(argv[2]);

    if(client.call(srv))
    {
        ROS_INFO("line deliver successful");
    }else{
        ROS_ERROR("Failed to call service");
    }

    return 0;
}