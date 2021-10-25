/*
 * @Author: your name
 * @Date: 2021-10-20 19:18:00
 * @LastEditTime: 2021-10-20 19:22:03
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/delphi_driver/kvaser_interface/src/delphi_kvaser_node.cpp
 */
#include "kvaser_interface/delphi_kvaser.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delphi_kvaser_node");
    ros::NodeHandle nh("~");
    DelphiKvaser delphi(nh);
    return 0;
}