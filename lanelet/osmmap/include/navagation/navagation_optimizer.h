/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 21:32:49
 * @LastEditTime: 2022-11-06 22:39:51
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/navagation/navagation_optimizer.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef NAVAGATION_OPTIMIZER_H_
#define NAVAGATION_OPTIMIZER_H_

#include "navagation.h"
#include "navagation_gnss.h"
#include "navagation_pcd.h"
#include "navagation_sim.h"
#include <iostream>
#include "glog/logging.h"

namespace navagation
{
class NavagationOptimizer
{
private:
    NavagationBase *navagation_ptr_;
    std::string mode_;
    ros::NodeHandle n_;
public:
    NavagationOptimizer(const std::string &mode, ros::NodeHandle &n);
    ~NavagationOptimizer();
};


}// namespace navagation

#endif