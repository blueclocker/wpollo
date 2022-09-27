/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-18 20:57:05
 * @LastEditTime: 2022-09-18 21:12:50
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/perception/pcl_test/src/pcl_test_node.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */

#include "pcl_test/pcl_test_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");

    ros::NodeHandle nh;

    PclTestCore core(nh);
    // core.Spin();
    return 0;
}
