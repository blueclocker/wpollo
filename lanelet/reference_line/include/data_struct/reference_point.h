/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-26 14:44:47
 * @LastEditTime: 2022-10-26 19:59:32
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/reference_line/include/data_struct/reference_point.h
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
 * @file reference_point.h
 **/

#pragma once

#include <string>
#include <vector>

#include "math/pnc_point.h"

namespace apollo {
namespace planning {

struct LaneWaypoint {
  LaneWaypoint() = default;
  // LaneWaypoint(LaneInfoConstPtr lane, const double s)
  //     : lane(CHECK_NOTNULL(lane)), s(s) {}
  // LaneWaypoint(LaneInfoConstPtr lane, const double s, const double l)
  //     : lane(CHECK_NOTNULL(lane)), s(s), l(l) {}
  LaneWaypoint(const double s)
      : s(s) {}
  LaneWaypoint(const double s, const double l)
      : s(s), l(l) {}
  // LaneInfoConstPtr lane = nullptr;
  double s = 0.0;
  double l = 0.0;
};

class MapPathPoint : public common::math::Vec2d {
 public:
  MapPathPoint() = default;
  MapPathPoint(const common::math::Vec2d& point, double heading)
      : Vec2d(point.x(), point.y()), heading_(heading) {}
  MapPathPoint(const common::math::Vec2d& point, double heading,
               LaneWaypoint lane_waypoint)
      : Vec2d(point.x(), point.y()), heading_(heading) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }
  MapPathPoint(const common::math::Vec2d& point, double heading,
               std::vector<LaneWaypoint> lane_waypoints)
      : Vec2d(point.x(), point.y()),
        heading_(heading),
        lane_waypoints_(std::move(lane_waypoints)) {}

  double heading() const { return heading_; }
  void set_heading(const double heading) { heading_ = heading; }

  const std::vector<LaneWaypoint>& lane_waypoints() const {
    return lane_waypoints_;
  }

  void add_lane_waypoint(LaneWaypoint lane_waypoint) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }
  void add_lane_waypoints(const std::vector<LaneWaypoint>& lane_waypoints) {
    lane_waypoints_.insert(lane_waypoints_.end(), lane_waypoints.begin(),
                           lane_waypoints.end());
  }

  void clear_lane_waypoints() { lane_waypoints_.clear(); }

  static void RemoveDuplicates(std::vector<MapPathPoint>* points);

  static std::vector<MapPathPoint> GetPointsFromSegment(
      const LaneSegment& segment);

  static std::vector<MapPathPoint> GetPointsFromLane(LaneInfoConstPtr lane,
                                                     const double start_s,
                                                     const double end_s);

  std::string DebugString() const;

 protected:
  double heading_ = 0.0;
  std::vector<LaneWaypoint> lane_waypoints_;
};

class ReferencePoint : public MapPathPoint {
 public:
  ReferencePoint() = default;

  ReferencePoint(const MapPathPoint& map_path_point, const double kappa,
                 const double dkappa);

  common::PathPoint ToPathPoint(double s) const;

  double kappa() const;
  double dkappa() const;

  std::string DebugString() const;

  static void RemoveDuplicates(std::vector<ReferencePoint>* points);

 private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

}  // namespace planning
}  // namespace apollo
