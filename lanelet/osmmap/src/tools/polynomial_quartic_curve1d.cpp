/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-20 22:09:40
 * @LastEditTime: 2022-11-20 22:42:54
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/tools/polynomial_quartic_curve1d.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "tools/polynomial_quartic_curve1d.h"

namespace math
{
PolynomialQuarticCurve1d::PolynomialQuarticCurve1d(const std::array<double, 3> &start, const std::array<double, 2> &end, const double param)
: PolynomialQuarticCurve1d(start[0], start[1], start[2], end[0], end[1], param)
{
}

PolynomialQuarticCurve1d::PolynomialQuarticCurve1d(const double x0, const double dx0, const double ddx0, 
                                                   const double dx1, const double ddx1, const double param)
{
    param_ = param;
    start_condition_[0] = x0;
    start_condition_[1] = dx0;
    start_condition_[2] = ddx0;
    end_condition_[0] = dx1;
    end_condition_[1] = ddx1;
    ComputeCoefficients(x0, dx0, ddx0, dx1, ddx1, param);
}

double PolynomialQuarticCurve1d::Evaluate(const std::uint32_t order, const double param) const
{
    switch (order)
    {
        case 0:
            return (((coef_[4] * param + coef_[3]) * param + coef_[2]) * param + coef_[1]) * param + coef_[0];

        case 1:
            return ((4.0 * coef_[4] * param + 3.0 * coef_[3]) * param + 2.0 * coef_[2]) * param + coef_[1];

        case 2:
            return (12.0 * coef_[4] * param + 6.0 * coef_[3]) * param + 2.0 * coef_[2];
        
        case 3:
            return 24.0 * coef_[4] * param + 6.0 * coef_[3];

        case 4:
            return 24.0 * coef_[4];
        
        default:
            return 0.0;
    }
}

double PolynomialQuarticCurve1d::Coef(const size_t order) const
{
    CHECK_GT(5, order);
    return coef_[order];
}

void PolynomialQuarticCurve1d::ComputeCoefficients(const double x0, const double dx0, const double ddx0, 
                                                   const double dx1, const double ddx1, const double param)
{
    DCHECK(param, 0.0);

    coef_[0] = x0;
    coef_[1] = dx0;
    coef_[2] = 0.5 * ddx0;

    const double b0 = dx1 - ddx0 * param - dx0;
    const double b1 = ddx1 - ddx0;

    const double p2 = param * param;
    const double p3 = param * p2;

    coef_[3] = (3.0 * b0 - b1 * param) / (3.0 * p2);
    coef_[4] = (-2.0 * b0 + b1 * param) / (4.0 * p3);
}


}//namespace math
