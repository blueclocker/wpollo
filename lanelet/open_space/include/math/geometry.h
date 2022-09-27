/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-17 20:30:15
 * @LastEditTime: 2022-09-18 16:00:13
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /catkin_ws/src/open_space/include/math/geometry.h
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <cmath>
#include <limits>


namespace apollo {
namespace common {
// A point in the map reference frame. The map defines an origin, whose
// coordinate is (0, 0, 0).
// Most modules, including localization, perception, and prediction, generate
// results based on the map reference frame.
// Currently, the map uses Universal Transverse Mercator (UTM) projection. See
// the link below for the definition of map origin.
//   https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
// The z field of PointENU can be omitted. If so, it is a 2D location and we do
// not care its height.
struct PointENU {
  double x = std::numeric_limits<double>::quiet_NaN();;  // East from the origin, in meters.
  double y = std::numeric_limits<double>::quiet_NaN();;  // North from the origin, in meters.
  double z = 0.0;  // Up from the WGS-84 ellipsoid, in
                                          // meters.
  void set_x(double a) {x = a;}
  void set_y(double a) {y = a;}
  void set_z(double a) {z = a;}
};

// A point in the global reference frame. Similar to PointENU, PointLLH allows
// omitting the height field for representing a 2D location.
struct PointLLH {
  // Longitude in degrees, ranging from -180 to 180.
  double lon = std::numeric_limits<double>::quiet_NaN();;
  // Latitude in degrees, ranging from -90 to 90.
  double lat = std::numeric_limits<double>::quiet_NaN();;
  // WGS-84 ellipsoid height in meters.
  double height = 0.0;
};

// A general 2D point. Its meaning and units depend on context, and must be
// explained in comments.
struct Point2D {
  double x = std::numeric_limits<double>::quiet_NaN();;
  double y = std::numeric_limits<double>::quiet_NaN();;
};

// A general 3D point. Its meaning and units depend on context, and must be
// explained in comments.
struct Point3D {
  double x = std::numeric_limits<double>::quiet_NaN();;
  double y = std::numeric_limits<double>::quiet_NaN();;
  double z = std::numeric_limits<double>::quiet_NaN();;
};

// A unit quaternion that represents a spatial rotation. See the link below for
// details.
//   https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
// The scalar part qw can be omitted. In this case, qw should be calculated by
//   qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
struct Quaternion {
  double qx = std::numeric_limits<double>::quiet_NaN();;
  double qy = std::numeric_limits<double>::quiet_NaN();;
  double qz = std::numeric_limits<double>::quiet_NaN();;
  double qw = std::numeric_limits<double>::quiet_NaN();;
};

// A general polygon, points are counter clockwise
struct Polygon {
  Point3D point;
};

}
}
#endif