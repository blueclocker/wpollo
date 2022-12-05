/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-20 15:17:53
 * @LastEditTime: 2022-11-20 15:55:21
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/tools/polynomial_cubic_curve1d.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef POLYNOMIAL_CUBIC_CURVE1D_H_
#define POLYNOMIAL_CUBIC_CURVE1D_H_

#include "polynomial_curve1d.h"
#include <array>

namespace math
{
class PolynomialCubicCurve1d : public PolynomialCurve1d
{
private:
    std::array<double, 4> coef_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 3> start_condition_ = {0.0, 0.0, 0.0};
    double end_condition_ = 0.0;
    void ComputeCoefficients(const double x0, const double dx0, const double ddx0, 
                             const double x1, const double param);
public:
    PolynomialCubicCurve1d() = default;
    //f(x = 0) = x0
    //f'(x = 0) = dx0
    //f''(x = 0) = ddx0
    //f(x = param) = x1
    PolynomialCubicCurve1d(const std::array<double, 3> &start, const double end, const double param);
    PolynomialCubicCurve1d(const double x0, const double dx0, const double ddx0, 
                           const double x1, const double param);
    virtual ~PolynomialCubicCurve1d() = default;
    double Evaluate(const std::uint32_t order, const double param) const override;
    double ParamLength() const override {return param_;}
    double Coef(const size_t order) const override;
    size_t Order() const override {return 3;}
};





}// namespace math


#endif