/*
 * @Author: wpbit
 * @Date: 2021-09-26 14:30:01
 * @LastEditTime: 2021-09-29 21:33:31
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/calibration/src/radar_esr_node.cpp
 */
#include "calibration/radar_esr.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_calibration");
    ros::NodeHandle nh("~");
    RadarCalibration radar_cal(nh);
    return 0;
}
