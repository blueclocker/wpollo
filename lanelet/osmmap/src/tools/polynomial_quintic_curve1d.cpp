/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-20 15:57:40
 * @LastEditTime: 2022-11-20 22:34:41
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/tools/polynomial_quintic_curve1d.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "tools/polynomial_quintic_curve1d.h"

namespace math
{
PolynomialQuinticCurve1d::PolynomialQuinticCurve1d(const std::array<double, 3> &start, const std::array<double,3> &end, const double param)
: PolynomialQuinticCurve1d(start[0], start[1], start[2], end[0], end[1], end[2], param)
{
}

PolynomialQuinticCurve1d::PolynomialQuinticCurve1d(const double x0, const double dx0, const double ddx0, 
                           const double x1, const double dx1, const double ddx1, const double param)
{
    start_condition_[0] = x0;
    start_condition_[1] = dx0;
    start_condition_[2] = ddx0;
    end_condition_[0] = x1;
    end_condition_[1] = dx1;
    end_condition_[2] = ddx1;
    param_ = param;
    ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
}

double PolynomialQuinticCurve1d::Evaluate(const std::uint32_t order, const double param) const
{
    switch (order)
    {
        case 0:
            return ((((coef_[5] * param + coef_[4]) * param + coef_[3]) * param + coef_[2]) * param + coef_[1]) * param + coef_[0];
        
        case 1: 
            return (((5.0 * coef_[5] * param + 4.0 * coef_[4]) * param + 3.0 * coef_[3]) * param + 2.0 * coef_[2]) * param + coef_[1];
        
        case 2: 
            return (((20.0 * coef_[5] * param + 12.0 * coef_[4]) * param) + 6.0 * coef_[3]) * param + 2.0 * coef_[2];

        case 3: 
            return (60.0 * coef_[5] * param + 24.0 * coef_[4]) * param + 6.0 * coef_[3];
        
        case 4: 
            return 120.0 * coef_[5] * param + 24.0 * coef_[4];
        
        case 5: 
            return 120.0 * coef_[5];
        
        default:
            return 0;
    }
}

double PolynomialQuinticCurve1d::Coef(const size_t order) const
{
    CHECK_GT(6, order);
    return coef_[order];
}

void PolynomialQuinticCurve1d::ComputeCoefficients(const double x0, const double dx0, const double ddx0, 
                                const double x1, const double dx1, const double ddx1, const double param)
{
    DCHECK(param > 0.0);
    coef_[0] = x0;
    coef_[1] = dx0;
    coef_[2] = 0.5 * ddx0;
    const double p2 = param * param;
    const double p3 = param * p2;

    const double c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * param - x0) / p3;
    const double c1 = (dx1 - ddx0 * param - dx0) / p2;
    const double c2 = (ddx1 - ddx0) / param;

    coef_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
    coef_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / param;
    coef_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
}



}//namespace math