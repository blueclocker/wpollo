/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-20 15:08:19
 * @LastEditTime: 2022-11-20 15:40:08
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/tools/curve1d.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef CURVE1D_H_
#define CURVE1D_H_

#include <math>
#include <iostream>
#include <glog/logging.h>
#include <gflags/gflags.h>

namespace math
{
class Curve1d
{
private:
    
public:
    Curve1d() = default;
    virtual ~Curve1d() = default;
    virtual double Evaluate(const std::uint32_t order, const double param) const = 0;
    virtual double ParamLength() const = 0;
};



}//namespace math

#endif