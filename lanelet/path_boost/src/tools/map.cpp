/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-31 20:40:06
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 21:46:17
 * @FilePath: /wpollo/src/lanelet/path_boost/src/tools/map.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "path_boost/tools/map.h"
#include <glog/logging.h>

namespace PathBoostNS
{
Map::Map(const grid_map::GridMap &grid_map) :
    maps(grid_map) {
    if (!grid_map.exists("distance")) {
        LOG(ERROR) << "grid map must contain 'distance' layer";
    }
}

double Map::getObstacleDistance(const Eigen::Vector2d &pos) const {
    if (maps.isInside(pos)) {
        return this->maps.atPosition("distance", pos, grid_map::InterpolationMethods::INTER_LINEAR);
    } else {
        return 0.0;
    }
}

bool Map::isInside(const Eigen::Vector2d &pos) const {
    return maps.isInside(pos);
}

}//namespace PathBoostNS