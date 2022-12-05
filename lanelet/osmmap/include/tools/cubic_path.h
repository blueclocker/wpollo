/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-21 19:27:07
 * @LastEditTime: 2022-11-22 15:21:24
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/tools/cubic_path.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef CUBIC_PATH_
#define CUBIC_PATH_

#include <iostream>
#include <cmath>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vector>
#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace math
{
class CubicLine
{
private:
    std::vector<double> x_raw_;//每段累计长度, 从0开始, n_
    std::vector<double> y_raw_;//实际一维坐标，n_
    std::vector<double> x_diff_;//n_-1 ->h(i) = x_raw_(i+1) - x_raw(i)
    std::vector<double> y_diff_;//n_-1 ->y_diff_(i) = y_raw_(i+1) - y_raw(i)
    int n_;
    // s(x) = a + b*x + c*x^2 + d*x^3 
    std::vector<double> a_;//n_-1
    std::vector<double> b_;//n_-1
    std::vector<double> c_;//n_-1
    std::vector<double> d_;//n_-1
    double length_;//总长

    void Compute();
    int bisect(double t, int start, int end) const;
    int Bisect(double t) const;
public:
    CubicLine() = default;
    CubicLine(const std::vector<double> &x, const std::vector<double> &y);
    ~CubicLine() = default;
    double Calculatet(const double t) const;
    double CalculateDt(const double t) const;
    double CalculateDDt(const double t) const;
    double CalculateDDDt(const double t) const;
};

class CubicPath
{
private:
    CubicLine sx_;
    CubicLine sy_;
    CubicLine sz_;
    std::vector<double> s_;
    double CalculateAccumulateLength(const std::vector<double> &x, const std::vector<double> &y, 
                                     const std::vector<double> &z);
public:
    CubicPath(const std::vector<double> &x, const std::vector<double> &y, 
              const std::vector<double> &z);
    ~CubicPath() = default;
    double GetLength() const {return s_.back();} //总长
    std::array<double, 3> GetPosition(const double t) const;//位置
    double GetYaw(const double t) const;//航向角
    double GetKappa(const double t) const;//曲率
    double GetRdkappa(const double t) const;//曲率对弧长一阶导数
};









}


#endif