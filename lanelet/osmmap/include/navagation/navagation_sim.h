/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 21:32:49
 * @LastEditTime: 2022-11-06 22:37:58
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/navagation/navagation_sim.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef NAVAGATION_SIM_H_
#define NAVAGATION_SIM_H_

#include "navagation.h"
#include <nav_msgs/Odometry.h>

namespace navagation
{
class NavagationSim : public NavagationBase
{
private:
    void SimCallback(const nav_msgs::Odometry::ConstPtr &msg);
public:
    NavagationSim(ros::NodeHandle &n);
    virtual ~NavagationSim() override = default;
    virtual void Process() override;
};


}//namespace navagation

#endif