/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-30 22:38:51
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 14:05:32
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/tools/tools.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef TOOLS_HPP_
#define TOOLS_HPP_

#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include <ctime>
#include "path_boost/data_struct/basic_struct.hpp"
#include "spline.h"

namespace PathBoostNS
{
// Set angle to -pi ~ pi
template <typename T>
T constrainAngle(T angle)
{
    if(angle < -M_PI)
    {
        angle += 2*M_PI;
        return constrainAngle(angle);
    }else if(angle > M_PI){
        angle -= 2*M_PI;
        return constrainAngle(angle);
    }else{
        return angle;
    }
}

// Time duration in seconds.
double time_s(const clock_t &begin, const clock_t &end);

// Time duration in ms.
double time_ms(const clock_t &begin, const clock_t &end);

// Output time duration.
void time_s_out(const clock_t &begin, const clock_t &end, const std::string &text);
void time_ms_out(const clock_t &begin, const clock_t &end, const std::string &text);

// Return true if a == b.
bool isEqual(double a, double b);

// Calculate heading for spline.
double getHeading(const tk::spline &xs, const tk::spline &ys, double s);

// Calculate curvature for spline.
double getCurvature(const tk::spline &xs, const tk::spline &ys, double tmp_s);

// Calculate distance between two points.
double distance(const BState &p1, const BState &p2);

// Coordinate transform.
BState local2Global(const BState &reference, const BState &target);
BState global2Local(const BState &reference, const BState &target);

// Find the closest point on spline.
BState getProjection(const tk::spline &xs,
                    const tk::spline &ys,
                    double target_x,
                    double target_y,
                    double max_s,
                    double start_s = 0.0);

BState getProjectionByNewton(const tk::spline &xs,
                            const tk::spline &ys,
                            double target_x,
                            double target_y,
                            double max_s,
                            double hint_s);

BState getDirectionalProjection(const tk::spline &xs,
                               const tk::spline &ys,
                               double target_x,
                               double target_y,
                               double angle,
                               double max_s,
                               double start_s = 0.0);

BState getDirectionalProjectionByNewton(const tk::spline &xs,
                                       const tk::spline &ys,
                                       double target_x,
                                       double target_y,
                                       double angle,
                                       double max_s,
                                       double hint_s);

}//namespace PathBoostNS

#endif