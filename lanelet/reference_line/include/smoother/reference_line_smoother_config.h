/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-26 15:17:33
 * @LastEditTime: 2022-10-26 16:23:26
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/reference_line/include/smoother/reference_line_smoother_config.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef REFERENCE_LINE_SMOOTHER_CONFIG_H_
#define REFERENCE_LINE_SMOOTHER_CONFIG_H_

#include <iostream>
#include "math/cos_theta_smoother_config.h"
#include "math/fem_pos_deviation_smoother_config.h"

namespace apollo
{
struct QpSplineSmootherConfig {
  uint32_t spline_order = 5;
  double max_spline_length = 25;
  double regularization_weight = 0.1;
  double second_derivative_weight = 0.0;
  double third_derivative_weight = 100;
}

struct SpiralSmootherConfig {
  // The max deviation of spiral reference line smoother.
  double max_deviation = 0.1;

  // The piecewise length of spiral smoother.
  double piecewise_length = 10.0;

  // The iteration num of spiral reference line smoother.";
  uint32_t max_iteration = 1000;

  // The desired convergence tol for spiral opt;
  double opt_tol = 1.0e-8;

  // The acceptable convergence tol for spiral opt
  double opt_acceptable_tol = 1e-6;

  // The number of acceptable iters before termination for spiral opt;
  uint32_t opt_acceptable_iteration = 15;

  // The weight of curve length term in objective function
  double weight_curve_length = 1.0;

  // The weight of kappa term in objective function
  double weight_kappa = 1.0;

  // The weight of dkappa term in objective function
  double weight_dkappa = 100.0;
}

enum SmoothingMethod {
  NOT_DEFINED = 0,
  COS_THETA_SMOOTHING = 1,
  FEM_POS_DEVIATION_SMOOTHING = 2
}

struct DiscretePointsSmootherConfig {
   SmoothingMethod smoothing_method = SmoothingMethod::FEM_POS_DEVIATION_SMOOTHING;
   CosThetaSmootherConfig cos_theta_smoothing;
   FemPosDeviationSmootherConfig fem_pos_deviation_smoothing;

//   oneof SmootherConfig {
//     CosThetaSmootherConfig cos_theta_smoothing = 4;
//     FemPosDeviationSmootherConfig fem_pos_deviation_smoothing = 5;
//   }
}

struct ReferenceLineSmootherConfig {
  // The output resolution for discrete point smoother reference line is
  // directly decided by max_constraint_interval
  double max_constraint_interval = 5;
  double longitudinal_boundary_bound = 1.0;
  double max_lateral_boundary_bound = 0.5;
  double min_lateral_boundary_bound = 0.2;
  // The output resolution for qp smoother reference line.
  uint32_t num_of_total_points = 500;
  double curb_shift = 0.2;
  double lateral_buffer = 0.2;
  // The output resolution for spiral smoother reference line.
  double resolution = 0.02;
//   oneof SmootherConfig {
//     QpSplineSmootherConfig qp_spline = 20;
//     SpiralSmootherConfig spiral = 21;
//     DiscretePointsSmootherConfig discrete_points = 22;
//   }
  DiscretePointsSmootherConfig discrete_points;
}


}
#endif