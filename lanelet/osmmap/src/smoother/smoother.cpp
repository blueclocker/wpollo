/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-08 14:20:14
 * @LastEditTime: 2022-11-08 20:22:34
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/smoother/smoother.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "smoother/smoother.h"
#include <chrono>


namespace apollo
{
namespace planning
{
PathSmoother::PathSmoother(const std::vector<map::centerway::CenterPoint3D> &path, const double delta_s) : 
path_raw_(path), delta_s_(delta_s)
{
    plan::Spline2D csp_obj(path_raw_);
    for(double i = 0; i < csp_obj.s.back(); i += delta_s_)
    {
        std::array<double, 3> point = csp_obj.calc_postion(i);
        x_list_.emplace_back(point[0]);
        y_list_.emplace_back(point[1]);
        z_list_.emplace_back(point[2]);
        s_list_.emplace_back(i);
        angle_list_.emplace_back(csp_obj.calc_yaw(i));
        k_list_.emplace_back(csp_obj.calc_curvature(i));
        if(i >= 15) break;
    }
}

PathSmoother::~PathSmoother()
{
}

bool PathSmoother::Process(const std::array<double, 3>& init_state, std::vector<map::centerway::CenterPoint3D> &result_path)
{
    std::array<double, 5> w = { 1.0,
                                100.0 * std::fmax(2.0, 5.0), 
                                // speed表示沿着参考线的速度。
                                // 速度越大，dl的权重越大，即尽量减小横向速度。
                                1000.0,
                                10000.0, 
                                0.0};

    std::vector<std::pair<double, double>> path_boundary;
    for(int i = 0; i < x_list_.size(); ++i)
    {
        path_boundary.emplace_back(std::make_pair(-1.5, 1.5));
    }

    // if the path_boundary is normal, it is possible to have less than 2 points
    // skip path boundary of this kind
    if (x_list_.size() < 2) return false;
    int max_iter = 4000;
    CHECK_GT(x_list_.size(), 1);

    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;

    std::array<double, 3> end_state = {0.0, 0.0, 0.0};

    // if (!FLAGS_enable_force_pull_over_open_space_parking_test) 
    // {
    //     // pull over scenario
    //     // set end lateral to be at the desired pull over destination
    //     const auto& pull_over_status =
    //         PlanningContext::Instance()->planning_status().pull_over();
    //     if (pull_over_status.has_position() &&
    //         pull_over_status.position().has_x() &&
    //         pull_over_status.position().has_y() &&
    //         path_boundary.label().find("pullover") != std::string::npos) 
    //         {
    //             common::SLPoint pull_over_sl;
    //             reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
    //             end_state[0] = pull_over_sl.l();
    //         }
    // }

    // 下面的公式是计算曲率，为什么变量名会是横向加速度？
    const double lat_acc_bound = std::tan(8.20304748437 / 16.0) / 2.8448;
        // std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) / veh_param.wheel_base();
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < k_list_.size(); ++i) 
    {
        double kappa = k_list_[i];
        // 看来ddl表示车辆的转向曲率与参考线曲率的差。
        ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }

    // 为什么第一个参数特意输入了当前沿着参考线的速度？
    bool res_opt = OptimizePath(init_state, end_state, path_boundary,
                                ddl_bounds, w, &opt_l, &opt_dl, &opt_ddl, max_iter);

    if (res_opt) 
    {
        std::cout << "osqp successed!!!" << std::endl;
        //输出规划结果 path_result
        //(s, opt_l) -> (x, y)
        CHECK_EQ(s_list_.size(), opt_l.size());
        for(int i = 0; i < s_list_.size(); ++i)
        {
            map::centerway::CenterPoint3D point;
            point.x_ = x_list_[i] - opt_l[i] * sin(angle_list_[i]);
            point.y_ = y_list_[i] + opt_l[i] * cos(angle_list_[i]);
            point.ele_ = z_list_[i];
            result_path.push_back(point);
        }
    }
    
    if (result_path.empty()) 
    {
        return false;
    }
    return true;
}

bool PathSmoother::OptimizePath(const std::array<double, 3>& init_state, const std::array<double, 3>& end_state,
                                const std::vector<std::pair<double, double>>& lat_boundaries,
                                const std::vector<std::pair<double, double>>& ddl_bounds,
                                const std::array<double, 5>& w, std::vector<double>* ptr_x,
                                std::vector<double>* ptr_dx, std::vector<double>* ptr_ddx,
                                const int max_iter)
{
    PiecewiseJerkPathProblem piecewise_jerk_problem(lat_boundaries.size(), delta_s_, init_state);

    // TODO(Hongyi): update end_state settings
    // 第一个array是weight，也就是说只对横向位置有要求吗？
    // 第二个array是终点状态，除了pullover，一般情况都是三个0，分别表示横向位置及其1,2阶导数。
    piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);
    // if (end_state[0] != 0) 
    // {
    //     // 如果靠边停车，那么将xref全都设置为停车位置的l。
    //     std::vector<double> x_ref(lat_boundaries.size(), end_state[0]);
    //     bool pull_over_type = std::sqrt(std::pow(init_state[0] - end_state[0], 2) + std::pow(init_state[1] - end_state[1], 2)) < 10;
    //     const double weight_x_ref = pull_over_type ? 200.0 : 10.0;
    //     piecewise_jerk_problem.set_x_ref(weight_x_ref, x_ref);
    // }

    piecewise_jerk_problem.set_weight_x(w[0]);
    piecewise_jerk_problem.set_weight_dx(w[1]);
    piecewise_jerk_problem.set_weight_ddx(w[2]);
    piecewise_jerk_problem.set_weight_dddx(w[3]);

    // 这个什么意思？
    piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

    auto start_time = std::chrono::system_clock::now();

    // 边界确实是用的path_bounds_decider里面的结果....但是这个方法没哟考虑车的尺寸吗？怎么保证无碰撞？
    piecewise_jerk_problem.set_x_bounds(lat_boundaries);
    // 默认值是2.
    piecewise_jerk_problem.set_dx_bounds(-FLAGS_lateral_derivative_bound_default,
                                         FLAGS_lateral_derivative_bound_default);
    // dd表示曲率，这里用的是车辆曲率与参考线曲率的差。
    // 问题：难道最小化的不是车辆的曲率，而是车辆曲率与参考线的差？看约束是怎么加的。
    piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);
    piecewise_jerk_problem.set_dddx_bound(FLAGS_lateral_jerk_bound);

    // Estimate lat_acc and jerk boundary from vehicle_params
    const double axis_distance = 2.8448;//veh_param.wheel_base;
    const double max_yaw_rate = 6.98131700798 / 16.0 / 2.0;
        //veh_param.max_steer_angle_rate / veh_param.steer_ratio / 2.0;
    const double jerk_bound = EstimateJerkBoundary(std::fmax(init_state[1], 1.0), axis_distance, max_yaw_rate);
    piecewise_jerk_problem.set_dddx_bound(jerk_bound);

    bool success = piecewise_jerk_problem.Optimize(max_iter);

    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    std::cout << "Path Optimizer used time: " << diff.count() * 1000 << " ms." << std::endl;

    if (!success)
    {
        std::cout << "piecewise jerk path optimizer failed" << std::endl;
        return false;
    }

    *ptr_x = piecewise_jerk_problem.opt_x();
    *ptr_dx = piecewise_jerk_problem.opt_dx();
    *ptr_ddx = piecewise_jerk_problem.opt_ddx();

    return true;
}

double PathSmoother::EstimateJerkBoundary(const double vehicle_speed, const double axis_distance,
                                          const double max_yaw_rate) const 
{
  return max_yaw_rate / axis_distance / vehicle_speed;
}


}//namespace planning

}// namespace apollo