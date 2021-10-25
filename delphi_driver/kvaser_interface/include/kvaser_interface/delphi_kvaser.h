/*
 * @Author: wpbit
 * @Date: 2021-09-16 21:18:52
 * @LastEditTime: 2021-10-20 19:28:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/delphi_driver/delphi/include/delphi/delphi.h
 */
#ifndef DELPHI_KVASER_H_
#define DELPHI_KVASER_H_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <can_msgs/delphi_msg.h>
#include <can_msgs/delphi_msges.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "math.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#define pi 3.1415926

class DelphiKvaser
{
private:
    ros::Publisher pub;
    ros::Publisher pub_mark;
    float db_group[64];
    can_msgs::delphi_msges msg_out;
    std::vector<can_msgs::delphi_msg> msg_pre;
    can_msgs::delphi_msg del_msg_;
    std::vector<visualization_msgs::Marker> msg_marks;
    visualization_msgs::MarkerArray msg_markerarray;
    visualization_msgs::Marker msg_marker;
    void Callback(const can_msgs::Frame::ConstPtr &msg);
public:
    DelphiKvaser(ros::NodeHandle &nh_);
    ~DelphiKvaser();
};


#endif
