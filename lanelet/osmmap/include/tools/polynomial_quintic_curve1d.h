/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-20 15:18:14
 * @LastEditTime: 2022-11-20 16:04:02
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/tools/polynomial_quintic_curve1d.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef POLYNOMIAL_QUINTIC_CURVE1D_H_
#define POLYNOMIAL_QUINTIC_CURVE1D_H_

#include "polynomial_curve1d.h"
#include <array>

namespace math
{
class PolynomialQuinticCurve1d : public PolynomialCurve1d
{
private:
    std::array<double, 3> start_condition_ = {0.0, 0.0, 0.0};
    std::array<double, 3> end_condition_ = {0.0, 0.0, 0.0};
    std::array<double, 6> coef_= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    void ComputeCoefficients(const double x0, const double dx0, const double ddx0, 
                             const double x1, const double dx1, const double ddx1, const double param);
public:
    PolynomialQuinticCurve1d() = default;
    PolynomialQuinticCurve1d(const std::array<double, 3> &start, const std::array<double,3> &end, const double param);
    PolynomialQuinticCurve1d(const double x0, const double dx0, const double ddx0, 
                             const double x1, const double dx1, const double ddx1, const double param);
    virtual ~PolynomialQuinticCurve1d() = default;
    double Evaluate(const std::uint32_t order, const double param) const override;
    double ParamLength() const override {return param_;}
    double Coef(const size_t order) const override;
    size_t Order() const override {return 5;}
};



}//namespace math

#endif