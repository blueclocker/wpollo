/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-30 22:00:06
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-30 22:08:04
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/config/config_flags.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CONFIG_FLAGS_HPP_
#define CONFIG_FLAGS_HPP_

#include <gflags/gflags.h>

DECLARE_double(car_width);

DECLARE_double(car_length);

DECLARE_double(safety_margin);

DECLARE_double(wheel_base);

DECLARE_double(rear_length);

DECLARE_double(front_length);

DECLARE_double(max_steering_angle);

DECLARE_string(smoothing_method);

DECLARE_string(tension_solver);

DECLARE_bool(enable_searching);

DECLARE_double(search_lateral_range);

DECLARE_double(search_longitudial_spacing);

DECLARE_double(search_lateral_spacing);

DECLARE_double(frenet_angle_diff_weight);

DECLARE_double(frenet_angle_diff_diff_weight);

DECLARE_double(frenet_deviation_weight);

DECLARE_double(cartesian_curvature_weight);

DECLARE_double(cartesian_curvature_rate_weight);

DECLARE_double(cartesian_deviation_weight);

DECLARE_double(tension_2_deviation_weight);

DECLARE_double(tension_2_curvature_weight);

DECLARE_double(tension_2_curvature_rate_weight);

DECLARE_bool(enable_simple_boundary_decision);

DECLARE_string(optimization_method);

DECLARE_double(K_curvature_weight);

DECLARE_double(K_curvature_rate_weight);

DECLARE_double(K_deviation_weight);

DECLARE_double(KP_curvature_weight);

DECLARE_double(KP_curvature_rate_weight);

DECLARE_double(KP_deviation_weight);

DECLARE_double(KP_slack_weight);

DECLARE_double(expected_safety_margin);

DECLARE_bool(constraint_end_heading);

DECLARE_bool(enable_exact_position);

DECLARE_double(output_spacing);

DECLARE_double(search_obstacle_cost);

DECLARE_double(search_deviation_cost);

DECLARE_double(epsilon);

DECLARE_bool(enable_dynamic_segmentation);

DECLARE_double(precise_planning_length);

DECLARE_bool(rough_constraints_far_away);

#endif