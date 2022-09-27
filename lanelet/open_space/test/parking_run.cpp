/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-20 22:04:33
 * @LastEditTime: 2022-09-20 22:06:06
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/open_space/test/parking_run.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "parkingflow.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking");
    ros::NodeHandle nh;
    apollo::planning::ParkingFlow *flow = new apollo::planning::ParkingFlow(nh);
    flow->init();
    flow->run();

    return 0;
}