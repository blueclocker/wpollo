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
