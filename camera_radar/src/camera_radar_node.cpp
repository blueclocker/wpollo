/*
 * @Author: wpbit
 * @Date: 2021-09-08 17:02:30
 * @LastEditTime: 2021-09-30 09:33:01
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/camera_radar/src/camera_radar_node.cpp
 */
#include "camera_radar/camera_radar.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_radar");
    ros::NodeHandle nh_space("~");
    CameraRadarCore core(nh_space);
    return 0;
}
