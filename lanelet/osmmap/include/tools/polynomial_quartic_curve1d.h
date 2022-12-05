/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-20 22:08:59
 * @LastEditTime: 2022-11-20 22:29:51
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/tools/polynomial_quartic_curve1d.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef POLYNOMIAL_QUARTIC_CURVE1D_H_
#define POLYNOMIAL_QUARTIC_CURVE1D_H_

#include "polynomial_curve1d.h"
#include <array>

namespace math
{
class PolynomialQuarticCurve1d : public PolynomialCurve1d
{
private:
    std::array<double, 3> start_condition_ = {0.0, 0.0, 0.0};
    std::array<double, 2> end_condition_ = {0.0, 0.0};
    std::array<double, 5> coef_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    void ComputeCoefficients(const double x0, const double dx0, const double ddx0, 
                             const double dx1, const double ddx1, const double param);
public:
    PolynomialQuarticCurve1d() = default;
    PolynomialQuarticCurve1d(const std::array<double, 3> &start, const std::array<double, 2> &end, const double param);
    PolynomialQuarticCurve1d(const double x0, const double dx0, const double ddx0, 
                             const double dx1, const double ddx1, const double param);
    virtual ~PolynomialQuarticCurve1d() = default;
    double Evaluate(const std::uint32_t order, const double param) const override;
    double ParamLength() const override {return param_;}
    double Coef(const size_t order) const override;
    size_t Order() const override {return 4;}
};




}//namespace math


#endif