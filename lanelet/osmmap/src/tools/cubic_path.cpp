/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-21 19:27:16
 * @LastEditTime: 2022-11-22 15:39:16
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/tools/cubic_path.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "tools/cubic_path.h"


namespace math
{
CubicLine::CubicLine(const std::vector<double> &x, const std::vector<double> &y)
{
    CHECK_EQ(x.size(), y.size());

    n_ = x.size();
    for(int i = 1; i < n_; ++i)
    {
        x_diff_.emplace_back(x[i] - x[i-1]);
        y_diff_.emplace_back(y[i] - y[i-1]);
    }
    x_raw_ = std::move(x);
    y_raw_ = std::move(y);
    length_ = x_raw_.back();
    Compute();
}

void CubicLine::Compute()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_, n_);// n_ * n_
    A(0, 0) = 1;
    for(int i = 1; i < n_ - 1; ++i)
    {
        A(i, i-1) = x_diff_[i-1];
        A(i, i) = (x_diff_[i-1] + x_diff_[i]) * 2;
        A(i, i+1) = x_diff_[i];
    }
    A(n_-1, n_-1) = 1;

    Eigen::VectorXd B = Eigen::VectorXd::Zero(n_);// n_ * 1
    for(int i = 1; i < n_ - 1; ++i)
    {
        B(i) = 6.0 * (y_diff_[i] / x_diff_[i] - y_diff_[i-1] / x_diff_[i-1]);
    }

    Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);//n_ * 1

    //a, b, c, d
    for(int i = 0; i < n_ - 1; ++i)
    {
        a_.emplace_back(y_raw_[i]);
        b_.emplace_back(y_diff_[i] / x_diff_[i] - 0.5 * x_diff_[i] * c_eigen(i) - x_diff_[i] * (c_eigen(i+1) - c_eigen(i)) / 6.0);
        c_.emplace_back(c_eigen(i) / 2.0);
        d_.emplace_back((c_eigen[i+1] - c_eigen[i]) / (6.0 * x_diff_[i]));
    }
}

int CubicLine::bisect(double t, int start, int end) const
{
    int mid = (start + end) / 2;
    if (t == x_raw_[mid] || end - start <= 1){
      return mid;
    }else if (t > x_raw_[mid]){
      return bisect(t, mid, end);
    }else{
      return bisect(t, start, mid);
    }
}

int CubicLine::Bisect(double t) const
{
    int l = 0;
    int r = n_ - 1;
    while(l < r)
    {
        int m = l + (r - l) / 2;
        if(x_raw_[m] <= t)
        {
            l = m + 1;
        }else{
            r = m;
        }
    }
    return l-1;
}

double CubicLine::Calculatet(const double t) const
{
    if(t < 0 || t > length_)
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    // int index = bisect(t, 0, n_);
    int index = Bisect(t);
    double dx = t - x_raw_[index];
    return ((d_[index] * dx + c_[index]) * dx + b_[index]) * dx + a_[index];
}

double CubicLine::CalculateDt(const double t) const
{
    if(t < 0 || t > length_)
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int index = Bisect(t);
    double dx = t - x_raw_[index];
    return (3.0 * d_[index] * dx + 2.0 * c_[index]) * dx + b_[index];
}

double CubicLine::CalculateDDt(const double t) const
{
    if(t < 0 || t > length_)
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int index = Bisect(t);
    double dx = t - x_raw_[index];
    return 6.0 * d_[index] * dx + 2.0 * c_[index];
}

double CubicLine::CalculateDDDt(const double t) const
{
    if(t < 0 || t > length_)
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int index = Bisect(t);
    double dx = t - x_raw_[index];
    return 6.0 * d_[index];
}

CubicPath::CubicPath(const std::vector<double> &x, const std::vector<double> &y, 
                     const std::vector<double> &z)
{
    CalculateAccumulateLength(x, y, z);
    sx_ = CubicLine(s_, x);
    sy_ = CubicLine(s_, y);
    sz_ = CubicLine(s_, z);
}

double CubicPath::CalculateAccumulateLength(const std::vector<double> &x, const std::vector<double> &y, 
                                            const std::vector<double> &z)
{
    CHECK_EQ(x.size(), y.size());
    CHECK_EQ(y.size(), z.size());

    std::vector<double> dx;
    std::vector<double> dy;
    std::vector<double> dz;
    for(int i = 1; i < x.size(); ++i)
    {
        dx.emplace_back(x[i] - x[i-1]);
        dy.emplace_back(y[i] - y[i-1]);
        dz.emplace_back(z[i] - z[i-1]);
    }

    s_.emplace_back(0);
    double sum = 0;
    for (int i = 0; i < dx.size(); i++)
    {
        sum += std::sqrt(dx[i] * dx[i] + dy[i] * dy[i] + dz[i] * dz[i]);
        s_.emplace_back(sum);
    }
    return sum;
}

std::array<double, 3> CubicPath::GetPosition(const double t) const
{
    double x = sx_.Calculatet(t);
    double y = sy_.Calculatet(t);
    double z = sz_.Calculatet(t);
    return {{x, y, z}};
}

double CubicPath::GetYaw(const double t) const
{
    double dx = sx_.CalculateDt(t);
    double dy = sy_.CalculateDt(t);
    return std::atan2(dy, dx);
}

double CubicPath::GetKappa(const double t) const
{
    double dx = sx_.CalculateDt(t);
    double ddx = sx_.CalculateDDt(t);
    double dy = sy_.CalculateDt(t);
    double ddy = sy_.CalculateDDt(t);
    return (ddy * dx - ddx * dy)/std::pow((dx * dx + dy * dy), 1.5);
}

double CubicPath::GetRdkappa(const double t) const
{
    double dx = sx_.CalculateDt(t);
    double ddx = sx_.CalculateDDt(t);
    double dddx = sx_.CalculateDDDt(t);
    double dy = sy_.CalculateDt(t);
    double ddy = sy_.CalculateDDt(t);
    double dddy = sy_.CalculateDDDt(t);
    return ((dddy*dx - dddx*dy)*(dx*dx + dy*dy) - 3*(dx*ddx + dy*ddy)*(ddy*dx - ddx*dy))/std::pow((dx * dx + dy * dy), 3.0);
}


}//namespace math

