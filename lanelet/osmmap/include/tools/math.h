/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-22 19:24:48
 * @LastEditTime: 2022-09-22 22:04:13
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/math.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef MATH_H_
#define MATH_H_

#include <limits>
#include <utility>
#include <cmath>
#include <array>

namespace plan
{
namespace math
{
// Notations:
// s_condition = [s, s_dot, s_ddot]
// s: longitudinal coordinate w.r.t reference line.
// s_dot: ds / dt
// s_ddot: d(s_dot) / dt
// d_condition = [d, d_dot, d_ddot]
// d: lateral coordinate w.r.t. reference line
// d_dot: dd / dt
// d_ddot: d(d_dot) / dt
// l: the same as d.
// d_prime = dd / ds = (dd / dt) / (ds / dt)
// d_pprime: d(d_prime) / ds
// frent <- -> Cartesian
class CartesianFrenetConverter
{
private:
    /* data */
public:
    CartesianFrenetConverter() = delete;
    ~CartesianFrenetConverter();
    
    // x : Cartesian坐标系下的车辆横坐标位置
    // y : Cartesian坐标系下的车辆纵坐标位置
    // v : Cartesian坐标系下的线速度大小
    // a : Cartesian坐标系下的加速度
    // theta : 为方位角，即全局坐标系下的朝向
    // kappa : 曲率
    // rs : 投影点的弧长
    // rx : 投影点P点在Cartesian坐标系下的x坐标
    // ry : 投影点P点在Cartesian坐标系下的y坐标
    // rtheta : 投影点P点在Cartesian坐标系下的朝向角
    // rkappa : 曲率
    // rdkappa : 曲率对弧长s的一阶导数
    // 返回:
    // ptr_s_condition : frenet坐标系的s, ds, dds(对t求导)
    // ptr_d_condition : frenet坐标系的d, dd, ddd(对t求导)
    // ptr_d_s : d关于s的导数, 导导数(对s求导)
    static void cartesian_to_frenet(const double rs, const double rx,
                                    const double ry, const double rtheta,
                                    const double rkappa, const double rdkappa,
                                    const double x, const double y,
                                    const double v, const double a,
                                    const double theta, const double kappa,
                                    std::array<double, 3>* const ptr_s_condition,
                                    std::array<double, 3>* const ptr_d_condition,
                                    std::array<double, 2>* const ptr_d_s);

    // rs : 投影点的弧长
    // rx : 投影点P点在Cartesian坐标系下的x坐标
    // ry : 投影点P点在Cartesian坐标系下的y坐标
    // rtheta : 投影点P点在Cartesian坐标系下的朝向角
    // x : Cartesian坐标系下的车辆横坐标位置
    // y : Cartesian坐标系下的车辆纵坐标位置
    // 返回:
    // ptr_s : 为纵向位移，即Frenet纵坐标
    // ptr_d : 横向位移, 即Frenet横坐标
    static void cartesian_to_frenet(const double rs, const double rx,
                                    const double ry, const double rtheta,
                                    const double x, const double y,
                                    double* ptr_s, double* ptr_d);
    
    // rs : 投影点P的弧长
    // rx : 投影点P点在Cartesian坐标系下的x坐标
    // ry : 投影点P点在Cartesian坐标系下的y坐标
    // rtheta : 投影点P点在Cartesian坐标系下的朝向角
    // rkappa : 曲率
    // rdkappa : 曲率对弧长s的一阶导数
    // s_condition : 为纵向位移，即Frenet纵坐标, 纵向速度, 纵向加速度
    // d_condition : 横向位移, 即Frenet横坐标, 横向速度, 横向加速度
    // 返回:
    // ptr_x : Cartesian坐标系下的车辆横坐标位置
    // ptr_y : Cartesian坐标系下的车辆纵坐标位置
    // ptr_theta : 为方位角，即全局坐标系下的朝向；
    // ptr_kappa : 曲率 
    // ptr_v : Cartesian坐标系下的线速度大小
    // ptr_a : Cartesian坐标系下的加速度
    static void frenet_to_cartesian(const double rs, const double rx,
                                    const double ry, const double rtheta,
                                    const double rkappa, const double rdkappa,
                                    const std::array<double, 3>& s_condition,
                                    const std::array<double, 3>& d_condition,
                                    double* const ptr_x, double* const ptr_y,
                                    double* const ptr_theta, double* const ptr_kappa, 
                                    double* const ptr_v, double* const ptr_a);

    // given sl point extract x, y, theta, kappa
    static double CalculateTheta(const double rtheta, const double rkappa,
                                 const double l, const double dl);

    static double CalculateKappa(const double rkappa, const double rdkappa,
                                 const double l, const double dl,
                                 const double ddl);

    static void CalculateCartesianPoint(const double rtheta, const double rx,
                                        const double ry, const double l,
                                        double* const ptr_x, double* const ptr_y);
};

// Normalize angle to [-PI, PI)
static double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

} // namespace math

} // namespace plan

#endif