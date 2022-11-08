/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-26 16:01:20
 * @LastEditTime: 2022-10-26 16:05:06
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/reference_line/include/math/cos_theta_smoother_config.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef COS_THETA_SMOOTHER_CONFIG_H_
#define COS_THETA_SMOOTHER_CONFIG_H_


namespace apollo
{
struct CosThetaSmootherConfig {
  double weight_cos_included_angle = 10000.0;
  double weight_anchor_points = 1.0;
  double weight_length = 1.0;

  // ipopt settings
  int print_level = 0;
  int max_num_of_iterations = 500;
  int acceptable_num_of_iterations = 15;
  double tol = 1e-8;
  double acceptable_tol = 1e-1;
  bool ipopt_use_automatic_differentiation = false;
}


}
#endif