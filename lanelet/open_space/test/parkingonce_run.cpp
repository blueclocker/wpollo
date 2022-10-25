/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-09 14:36:51
 * @LastEditTime: 2022-10-09 15:17:57
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/open_space/test/parkingonce_run.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "parkingonce.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parkingonce");
    ros::NodeHandle nh;
    apollo::planning::ParkingOnce parking(nh);
    // parking.run();

    return 0;
}