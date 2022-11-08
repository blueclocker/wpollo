/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-10-26 14:44:47
 * @LastEditTime: 2022-10-26 20:09:25
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/reference_line/src/data_struct/reference_point.cc
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
 * @file reference_point.cc
 **/

#include "data_struct/reference_point.h"

#include "absl/strings/str_cat.h"
#include "math/point_factory.h"

namespace apollo {
namespace planning {

namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
}  // namespace

std::vector<MapPathPoint> MapPathPoint::GetPointsFromSegment(
    const LaneSegment& segment) {
  return GetPointsFromLane(segment.lane, segment.start_s, segment.end_s);
}

std::vector<MapPathPoint> MapPathPoint::GetPointsFromLane(LaneInfoConstPtr lane,
                                                          const double start_s,
                                                          const double end_s) {
  std::vector<MapPathPoint> points;
  if (start_s >= end_s) {
    return points;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points.emplace_back(lane->points()[i], lane->headings()[i],
                          LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto& segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points.emplace_back(segment.start() + segment.unit_direction() *
                                                  (start_s - accumulate_s),
                            lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points.emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
  return points;
}

void MapPathPoint::RemoveDuplicates(std::vector<MapPathPoint>* points) {
  static constexpr double kDuplicatedPointsEpsilon = 1e-7;
  static constexpr double limit =
      kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  CHECK_NOTNULL(points);
  int count = 0;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

std::string MapPathPoint::DebugString() const {
  return absl::StrCat(
      "x = ", x_, "  y = ", y_, "  heading = ", heading_,
      "  lwp = "
      "{(",
      absl::StrJoin(lane_waypoints_, "), (", DebugStringFormatter()), ")}");
}

ReferencePoint::ReferencePoint(const MapPathPoint& map_path_point,
                               const double kappa, const double dkappa)
    : MapPathPoint(map_path_point), kappa_(kappa), dkappa_(dkappa) {}

common::PathPoint ReferencePoint::ToPathPoint(double s) const {
  return common::util::PointFactory::ToPathPoint(x(), y(), 0.0, s, heading(),
                                                 kappa_, dkappa_);
}

double ReferencePoint::kappa() const { return kappa_; }

double ReferencePoint::dkappa() const { return dkappa_; }

std::string ReferencePoint::DebugString() const {
  return absl::StrCat("{x: ", x(), ", y: ", y(), ", theta: ", heading(),
                      ", kappa: ", kappa(), ", dkappa: ", dkappa(), "}");
}

void ReferencePoint::RemoveDuplicates(std::vector<ReferencePoint>* points) {
  CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

}  // namespace planning
}  // namespace apollo
