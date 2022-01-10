/*
 * @Author: your name
 * @Date: 2022-01-10 10:20:45
 * @LastEditTime: 2022-01-10 10:51:38
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/mapclient.cpp
 */
#include <iostream>
#include <ros/ros.h>
#include "graph_tool/SetPoint.h"
#include "graph_tool/SetPath.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_client");
    ros::NodeHandle nh;

    ros::spin();
    return 0;
}