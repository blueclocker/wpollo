/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-26 16:01:32
 * @LastEditTime: 2022-10-26 16:08:31
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/reference_line/include/math/fem_pos_deviation_smoother_config.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef FEM_POS_DEVIATION_SMOOTHER_CONFIG_H_
#define FEM_POS_DEVIATION_SMOOTHER_CONFIG_H_


namespace apollo
{
struct FemPosDeviationSmootherConfig {
  double weight_fem_pos_deviation = 1.0e10;
  double weight_ref_deviation = 1.0;
  double weight_path_length = 1.0;
  bool apply_curvature_constraint = false;
  double weight_curvature_constraint_slack_var = 1.0e2;
  double curvature_constraint = 0.2;
  bool use_sqp = false;
  double sqp_ftol = 1e-4;
  double sqp_ctol = 1e-3;
  int sqp_pen_max_iter = 10;
  int sqp_sub_max_iter = 100;

  // osqp settings
  int max_iter = 500;
  // time_limit set to be 0.0 meaning no time limit
  double time_limit = 0.0;
  bool verbose = false;
  bool scaled_termination = true;
  bool warm_start = true;

  // ipopt settings
  int print_level = 0;
  int max_num_of_iterations = 500;
  int acceptable_num_of_iterations = 15;
  double tol = 1e-8;
  double acceptable_tol = 1e-1;
}

}
#endif