/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-20 15:08:38
 * @LastEditTime: 2022-11-20 15:24:47
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/tools/polynomial_curve1d.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef POLYNOMIAL_CURVE1D_H_
#define POLYNOMIAL_CURVE1D_H_

#include "curve1d.h"

namespace math
{
class PolynomialCurve1d : public Curve1d
{
protected:
    double param_;
public:
    PolynomialCurve1d() = default;
    virtual ~PolynomialCurve1d() = default;
    virtual double Coef(const size_t order) const = 0;
    virtual size_t Order() const = 0;
};




}//namespace math

#endif