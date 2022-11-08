/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-08 14:19:58
 * @LastEditTime: 2022-11-08 19:36:07
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/include/smoother/smoother.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef SMOOTHER_H_
#define SMOOTHER_H_

#include "osmmap/centerway.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "osmmap/cubic_spline.h"
#include "piecewise_jerk_path_problem.h"
#include "planning_gflags.h"

#include <utility>
#include <vector>

namespace apollo
{
namespace planning
{
class PathSmoother
{
private:
    std::vector<map::centerway::CenterPoint3D> path_raw_;
    std::vector<double> x_list_;
    std::vector<double> y_list_;
    std::vector<double> z_list_;
    std::vector<double> s_list_;
    std::vector<double> angle_list_;
    std::vector<double> k_list_;
    double delta_s_;
public:
    PathSmoother(const std::vector<map::centerway::CenterPoint3D> &path, const double delta_s = 0.2);
    ~PathSmoother();
    bool OptimizePath(const std::array<double, 3>& init_state,
                      const std::array<double, 3>& end_state,
                      const std::vector<std::pair<double, double>>& lat_boundaries,
                      const std::vector<std::pair<double, double>>& ddl_bounds,
                      const std::array<double, 5>& w, std::vector<double>* ptr_x,
                      std::vector<double>* ptr_dx, std::vector<double>* ptr_ddx,
                      const int max_iter);
    double EstimateJerkBoundary(const double vehicle_speed, const double axis_distance, const double max_yaw_rate) const;
    bool Process(const std::array<double, 3>& init_state, std::vector<map::centerway::CenterPoint3D> &result_path);
};







}// namespace planning

}//namespace appllo

#endif