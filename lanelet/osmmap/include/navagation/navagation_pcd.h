/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 19:32:28
 * @LastEditTime: 2022-11-06 21:41:51
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/navagation/navagation_pcd.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef NAVAGZTION_PCD_H_
#define NAVAGATION_PCD_H_

#include "navagation.h"
#include <nav_msgs/Odometry.h>

namespace navagation
{
class NavagationPcd : public NavagationBase
{
private:
    void PcdCallback(const nav_msgs::Odometry::ConstPtr &msg);
public:
    NavagationPcd(ros::NodeHandle &n);
    virtual ~NavagationPcd() override = default;
    virtual void Process() override;
};

}// namespace navagation

#endif