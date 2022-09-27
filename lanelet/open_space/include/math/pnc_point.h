/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-17 20:11:12
 * @LastEditTime: 2022-09-18 19:41:38
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/open_space/include/math/pnc_point.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef PNC_POINT_H_
#define PNC_POINT_H_

#include <string>
#include <iostream>

namespace apollo {
namespace common {
struct SLPoint {
  double s;
  double l;
  void set_s(double a) {s = a;}
  void set_l(double a) {l = a;}
};

struct FrenetFramePoint {
  double s;
  double l;
  double dl;
  double ddl;
  void set_s(double a) {s = a;}
  void set_l(double a) {l = a;}
  void set_dl(double a) {dl = a;}
  void set_ddl(double a) {ddl = a;}
};

struct SpeedPoint {
  double s;
  double t;
  // speed (m/s)
  double v;
  // acceleration (m/s^2)
  double a;
  // jerk (m/s^3)
  double da;
  void set_s(double a) {s = a;}
  void set_t(double a) {t = a;}
  void set_v(double a) {v = a;}
  void set_a(double a_) {a = a_;}
  void set_da(double a) {da = a;}
  void Clear()
  {
    s = 0;
    t = 0;
    v = 0;
    a = 0;
    da = 0;
  }
  bool has_v() const {return v != 0;}
  bool has_a() const {return a != 0;}
  bool has_da() const {return da != 0;}
};

struct PathPoint {
  // coordinates
  double x;
  double y;
  double z;

  // direction on the x-y plane
  double theta;
  // curvature on the x-y planning
  double kappa;
  // accumulated distance from beginning of the path
  double s;

  // derivative of kappa w.r.t s.
  double dkappa;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa;
  // The lane ID where the path point is on
  std::string lane_id;

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
  double x_derivative;
  double y_derivative;
  void set_x(double a) {x = a;}
  void set_y(double a) {y = a;}
  void set_z(double a) {z = a;}
  void set_theta(double a) {theta = a;}
  void set_kappa(double a) {kappa = a;}
  void set_s(double a) {s = a;}
  void set_dkappa(double a) {dkappa = a;}
  void set_ddkappa(double a) {ddkappa = a;}
};

struct Path {
  std::string name;
  PathPoint path_point;
  bool has_path_point() const {return path_point.lane_id != "";}
};

struct GaussianInfo {
  // Information of gaussian distribution
  double sigma_x;
  double sigma_y;
  double correlation;
  // Information of representative uncertainty area
  double area_probability;
  double ellipse_a;
  double ellipse_b;
  double theta_a;
};

struct TrajectoryPoint {
  // path point
  PathPoint path_point;
  // linear velocity
  double v;  // in [m/s]
  // linear acceleration
  double a;
  // relative time from beginning of the trajectory
  double relative_time;
  // longitudinal jerk
  double da;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer;

  // Gaussian probability information
  GaussianInfo gaussian_info;

  void set_v(double t) {v = t;}
  void set_a(double t) {a = t;}
  void set_relative_time(double t) {relative_time = t;}
  void set_da(double t) {da = t;}
  void set_steer(double t) {steer = t;}
  bool has_path_point() const {return path_point.lane_id != "";}
  PathPoint mutable_path_point() {return path_point;}
};

struct Trajectory {
  std::string name;
  TrajectoryPoint trajectory_point;
};

struct VehicleMotionPoint {
  // trajectory point
  TrajectoryPoint trajectory_point;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer;
};

struct VehicleMotion {
  std::string name;
  VehicleMotionPoint vehicle_motion_point;
};

}
}
#endif