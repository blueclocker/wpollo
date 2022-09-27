/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-22 23:06:54
 * @LastEditTime: 2022-09-25 19:58:41
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/plan/math_test.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "osmmap/math.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "math_test");
    ros::NodeHandle nh;

    double rs = 10.0;
    double rx = 0.0;
    double ry = 0.0;
    double rtheta = M_PI / 4.0;
    double rkappa = 0.1;
    double rdkappa = 0.01;
    double x = -1.0;
    double y = 1.0;
    double v = 2.0;
    double a = 0.0;
    double theta = M_PI / 3.0;
    double kappa = 0.11;

    std::array<double, 3> s_conditions;
    std::array<double, 3> d_conditions;
    std::array<double, 2> d_s;

    plan::math::CartesianFrenetConverter::cartesian_to_frenet(
        rs, rx, ry, rtheta, rkappa, rdkappa, x, y, v, a, theta, kappa,
        &s_conditions, &d_conditions, &d_s);
    plan::math::CartesianFrenetConverter::frenet_to_cartesian(
        rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x, &y, &theta, &kappa, &v, &a
    );
    std::cout << x << ", " << y << ", " << theta << std::endl;
    std::cout << kappa << ", " << v << ", " << a << std::endl;
    ros::spin();
    return 0;
}
