/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-17 13:55:02
 * @LastEditTime: 2022-09-17 14:09:24
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/open_space/include/constraint_checker/collision_checker.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <memory>
#include <vector>

#include "math/box2d.h"
#include "planning/obstacle.h"
#include "planning/reference_line_info.h"
#include "planning/discretized_trajectory.h"
#include "modules/planning/lattice/behavior/path_time_graph.h"

namespace apollo {
namespace planning {

class CollisionChecker {
 public:
  CollisionChecker(
      const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<common::PathPoint>& discretized_reference_line,
      const ReferenceLineInfo* ptr_reference_line_info,
      const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph);

  bool InCollision(const DiscretizedTrajectory& discretized_trajectory);

  static bool InCollision(const std::vector<const Obstacle*>& obstacles,
                          const DiscretizedTrajectory& ego_trajectory,
                          const double ego_length, const double ego_width,
                          const double ego_edge_to_center);

 private:
  void BuildPredictedEnvironment(
      const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<common::PathPoint>& discretized_reference_line);

  bool IsEgoVehicleInLane(const double ego_vehicle_s,
                          const double ego_vehicle_d);

  bool IsObstacleBehindEgoVehicle(
      const Obstacle* obstacle, const double ego_vehicle_s,
      const std::vector<apollo::common::PathPoint>& discretized_reference_line);

 private:
  const ReferenceLineInfo* ptr_reference_line_info_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles_;
};

}  // namespace planning
}  // namespace apollo
