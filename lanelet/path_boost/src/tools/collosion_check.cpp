/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-31 20:40:30
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 21:42:27
 * @FilePath: /wpollo/src/lanelet/path_boost/src/tools/collosion_check.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "path_boost/config/config_flags.hpp"
#include "path_boost/tools/collosion_check.h"

namespace PathBoostNS
{
CollisionChecker::CollisionChecker(const grid_map::GridMap &in_gm)
    : map_(in_gm),
      car_(FLAGS_car_width,
           fabs(FLAGS_rear_length),
           FLAGS_front_length)
{
}

bool CollisionChecker::isSingleStateCollisionFree(const BState &current) {
    // get the footprint circles based on current vehicle state in global frame
    std::vector<Circle> footprint =
        this->car_.getCircles(current);
    // footprint checking
    for (auto &circle_itr : footprint) {
        grid_map::Position pos(circle_itr.x,
                               circle_itr.y);
        // complete collision checking
        if (map_.isInside(pos)) {
            double clearance = this->map_.getObstacleDistance(pos);
            if (clearance < circle_itr.r) {  // collision
                // less than circle radius, collision
                return false;
            }
        } else {
            // beyond boundaries , collision
            return false;
        }
    }
    // all checked, current state is collision-free
    return true;
}

bool CollisionChecker::isSingleStateCollisionFreeImproved(const BState &current) {
    // get the bounding circle position in global frame
    Circle bounding_circle = this->car_.getBoundingCircle(current);

    grid_map::Position pos(bounding_circle.x,
                           bounding_circle.y);
    if (map_.isInside(pos)) {
        double clearance = this->map_.getObstacleDistance(pos);
        if (clearance < bounding_circle.r) {
            // the big circle is not collision-free, then do an exact
            // collision checking
            return (this->isSingleStateCollisionFree(current));
        } else { // collision-free
            return true;
        }
    } else {  // beyond the map boundary
        return false;
    }
}

}//namespace PathBoostNS