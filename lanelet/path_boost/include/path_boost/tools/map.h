/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-31 20:40:43
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 21:45:39
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/tools/map.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MAP_H_
#define MAP_H_

#include <iostream>
#include <cassert>
#include <stdexcept>
#include "Eigen/Core"
#include <grid_map_core/grid_map_core.hpp>


namespace PathBoostNS
{
class Map {
 public:
    Map() = delete;
    explicit Map(const grid_map::GridMap &grid_map);
    double getObstacleDistance(const Eigen::Vector2d &pos) const;
    bool isInside(const Eigen::Vector2d &pos) const;

 private:
    const grid_map::GridMap &maps;
};



}

#endif