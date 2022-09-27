/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-22 19:24:28
 * @LastEditTime: 2022-09-25 20:05:57
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/plan/math.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "osmmap/math.h"
#include <iostream>


namespace plan
{
namespace math
{
CartesianFrenetConverter::~CartesianFrenetConverter()
{
}

void CartesianFrenetConverter::cartesian_to_frenet( const double rs, const double rx,
                                                    const double ry, const double rtheta,
                                                    const double rkappa, const double rdkappa,
                                                    const double x, const double y,
                                                    const double v, const double a,
                                                    const double theta, const double kappa,
                                                    std::array<double, 3>* const ptr_s_condition,
                                                    std::array<double, 3>* const ptr_d_condition,
                                                    std::array<double, 2>* const ptr_d_s)
{
    double dx = x - rx;
    double dy = y - ry;
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double delta_theta = theta - rtheta;
    double cos_delta_theta = std::cos(delta_theta);
    double sin_delta_theta = std::sin(delta_theta);
    double tan_delta_theta = std::tan(delta_theta);
    //d
    ptr_d_condition->at(0) = std::copysign(std::sqrt(dx * dx + dy * dy), dy * cos_theta - dx * sin_theta);
    //dd
    ptr_d_condition->at(1) = v * sin_delta_theta;
    //ddd
    ptr_d_condition->at(2) = a * sin_delta_theta;
    double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
    //d关于s的导数
    ptr_d_s->at(0) = one_minus_kappa_r_d * tan_delta_theta;
    double kappa_r_d_prime = rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_s->at(0);
    //d关于s的导导数
    ptr_d_s->at(1) = -kappa_r_d_prime * tan_delta_theta + one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
                     (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);
    std::cout << ptr_d_s->at(1) << std::endl;
    //s
    ptr_s_condition->at(0) = rs;
    //ds 
    ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;
    double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
    //dds
    ptr_s_condition->at(2) = (a * cos_delta_theta - ptr_s_condition->at(1) * ptr_s_condition->at(1) * 
                              (ptr_d_s->at(0) * delta_theta_prime - kappa_r_d_prime)) / one_minus_kappa_r_d;

    //未解决，d_pprime != ptr_d_s->at(1)，反验证似乎对，可能double运算精度损失
    double d_pprime = (ptr_d_condition->at(2) * ptr_s_condition->at(1) - ptr_d_condition->at(1) * ptr_s_condition->at(2)) / 
                      (ptr_s_condition->at(1) * ptr_s_condition->at(1) * ptr_s_condition->at(1));    
    // double d_pprime = (ptr_d_condition->at(2) - ptr_d_s->at(0) * ptr_s_condition->at(2)) / 
                    //   (ptr_s_condition->at(1) * ptr_s_condition->at(1));  
    std::cout << d_pprime << std::endl;
}

void CartesianFrenetConverter::cartesian_to_frenet( const double rs, const double rx,
                                                    const double ry, const double rtheta,
                                                    const double x, const double y,
                                                    double* ptr_s, double* ptr_d)
{
    double dx = x - rx;
    double dy = y - ry;
    double cos_rtheta = std::cos(rtheta);
    double sin_rtheta = std::sin(rtheta);
    *ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), dy * cos_rtheta - dx * sin_rtheta);
    *ptr_s = rs;
}

void CartesianFrenetConverter::frenet_to_cartesian( const double rs, const double rx,
                                                    const double ry, const double rtheta,
                                                    const double rkappa, const double rdkappa,
                                                    const std::array<double, 3>& s_condition,
                                                    const std::array<double, 3>& d_condition,
                                                    double* const ptr_x, double* const ptr_y,
                                                    double* const ptr_theta, double* const ptr_kappa, 
                                                    double* const ptr_v, double* const ptr_a)
{
    double cos_rtheta = std::cos(rtheta);
    double sin_rtheta = std::sin(rtheta);
    //x
    *ptr_x = rx - sin_rtheta * d_condition[0];
    //y
    *ptr_y = ry + cos_rtheta * d_condition[0];
    double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];
    double tan_delta_theta = d_condition[1] / s_condition[1] / one_minus_kappa_r_d;
    double delta_theta = std::atan2(d_condition[1] / s_condition[1], one_minus_kappa_r_d);
    double cos_delta_theta = std::cos(delta_theta);
    //theta 
    *ptr_theta = NormalizeAngle(delta_theta + rtheta);
    double kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1] / s_condition[1];
    //kappa
    double d_pprime = (d_condition[2] * s_condition[1] - d_condition[1] * s_condition[2]) / 
                      (s_condition[1] * s_condition[1] * s_condition[1]);
    *ptr_kappa =((d_pprime + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta *
                 cos_delta_theta / one_minus_kappa_r_d + rkappa) * cos_delta_theta / one_minus_kappa_r_d;
    //v
    *ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * s_condition[1] * s_condition[1] 
                        + d_condition[1] * d_condition[1]);
    double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;
    //a
    *ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta + s_condition[1] * s_condition[1] / cos_delta_theta *
             (d_condition[1] / s_condition[1] * delta_theta_prime - kappa_r_d_prime);
}

double CartesianFrenetConverter::CalculateTheta(const double rtheta, const double rkappa,
                                                const double l, const double dl)
{
    return NormalizeAngle(rtheta + std::atan2(dl, 1 - l * rkappa));
}

double CartesianFrenetConverter::CalculateKappa(const double rkappa, const double rdkappa,
                                                const double l, const double dl,
                                                const double ddl)
{
    double denominator = (dl * dl + (1 - l * rkappa) * (1 - l * rkappa));
    if (std::fabs(denominator) < 1e-8) 
    {
        return 0.0;
    }
    denominator = std::pow(denominator, 1.5);
    const double numerator = rkappa + ddl - 2 * l * rkappa * rkappa -
                            l * ddl * rkappa + l * l * rkappa * rkappa * rkappa +
                            l * dl * rdkappa + 2 * dl * dl * rkappa;
    return numerator / denominator;
}

void CartesianFrenetConverter::CalculateCartesianPoint(const double rtheta, const double rx,
                                                       const double ry, const double l,
                                                       double* const ptr_x, double* const ptr_y)
{
    *ptr_x = rx - l * std::sin(rtheta);
    *ptr_y = ry - l * std::cos(rtheta);
}


} // namespace math
 
} // namespace plan
