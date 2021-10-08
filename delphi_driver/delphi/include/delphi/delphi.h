/*
 * @Author: wpbit
 * @Date: 2021-09-16 21:18:52
 * @LastEditTime: 2021-10-07 19:26:52
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/delphi_driver/delphi/include/delphi/delphi.h
 */
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <can_msgs/delphi_msg.h>
#include <can_msgs/delphi_msges.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

can_msgs::delphi_msges msg_out;
std::vector<can_msgs::delphi_msg> msg_pre;
ros::Publisher pub;
float db_group[64];
can_msgs::delphi_msg del_msg_;
