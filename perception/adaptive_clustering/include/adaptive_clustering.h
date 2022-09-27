/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-18 20:57:05
 * @LastEditTime: 2022-09-18 21:06:09
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/perception/adaptive_clustering/include/adaptive_clustering.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <iostream>

// can_msgs::delphi_msges msg_out;
// std::vector<can_msgs::delphi_msg> msg_pre;
ros::Publisher pub;
float db_group[64];
// can_msgs::delphi_msg del_msg_;
