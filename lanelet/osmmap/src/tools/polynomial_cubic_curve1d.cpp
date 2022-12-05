/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-20 15:20:29
 * @LastEditTime: 2022-11-20 22:42:39
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/tools/polynomial_cubic_curve1d.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "tools/polynomial_cubic_curve1d.h"

namespace math
{
PolynomialCubicCurve1d::PolynomialCubicCurve1d(const std::array<double, 3> &start, const double end, const double param)
: PolynomialCubicCurve1d(start[0], start[1], start[2], end, param)
{
}

PolynomialCubicCurve1d::PolynomialCubicCurve1d(const double x0, const double dx0, const double ddx0, 
                                               const double x1, const double param)
{
    ComputeCoefficients(x0, dx0, ddx0, x1, param);
    param_ = param;
    start_condition_[0] = x0;
    start_condition_[1] = dx0;
    start_condition_[2] = ddx0;
    end_condition_ = x1;
}

double PolynomialCubicCurve1d::Evaluate(const std::uint32_t order, const double param) const
{
    switch (order)
    {
        case 0 :
            return ((coef_[3] * param + coef_[2]) * param + coef_[1]) * param + coef_[0];

        case 1 :
            return (3.0 * coef_[3] * param + 2.0 * coef_[2]) * param + coef_[1];

        case 2 :
            return 6.0 * coef_[3] * param + 2.0 * coef_[2];
        
        case 3 : 
            return 6.0 * coef_[3];
        
        default:
            return 0.0;
    }
}

double PolynomialCubicCurve1d::Coef(const size_t order) const
{
    CHECK_GT(4, order);
    return coef_[order];
}

void PolynomialCubicCurve1d::ComputeCoefficients(const double x0, const double dx0, const double ddx0, 
                                                 const double x1, const double param)
{
    DCHECK(param > 0.0);
    const double p2 = param * param;
    const double p3 = param * p2;
    coef_[0] = x0;
    coef_[1] = dx0;
    coef_[2] = 0.5 * ddx0;
    coef_[3] = (x1 - x0 - dx0 * param - 0.5 * ddx0 * p2) / p3;
}



}//namespace math


