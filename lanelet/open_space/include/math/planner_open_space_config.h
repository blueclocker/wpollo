/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-17 14:19:38
 * @LastEditTime: 2022-09-17 19:45:06
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/open_space/include/math/planner_open_space_config.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef PLANNER_OPEN_SPACE_CONFIG_H_
#define PLANNER_OPEN_SPACE_CONFIG_H_

#include <iostream>
#include <cmath>

namespace apollo {
namespace planning {
struct WarmStartConfig {
  // Hybrid a star for warm start
  double xy_grid_resolution = 0.2;
  double phi_grid_resolution = 0.05;
  u_int64_t next_node_num = 10;
  double step_size = 0.5;
  double traj_forward_penalty = 0.0;
  double traj_back_penalty = 0.0;
  double traj_gear_switch_penalty = 10.0;
  double traj_steer_penalty = 100.0;
  double traj_steer_change_penalty = 10.0;
  // Grid a star for heuristic
  double grid_a_star_xy_resolution = 0.1;
  double node_radius = 0.5;
  // PiecewiseJerkSpeedConfig s_curve_config;
};

struct PlannerOpenSpaceConfig
{
  WarmStartConfig warm_start_config;
  // Dual Variable Warm Start
  // DualVariableWarmStartConfig dual_variable_warm_start_config = 3;
  // Distance Approach Configs
  // DistanceApproachConfig distance_approach_config = 4;
  // Iterative Anchoring Configs
  // IterativeAnchoringConfig iterative_anchoring_smoother_config = 5;
  // Trajectory PartitionConfig Configs
  // TrajectoryPartitionConfig trajectory_partition_config = 6;
  float delta_t = 1.0;
  double is_near_destination_threshold = 0.001;
  bool enable_check_parallel_trajectory = false;
  bool enable_linear_interpolation = false;
  double is_near_destination_theta_threshold = 0.05;
};

}
}
#endif