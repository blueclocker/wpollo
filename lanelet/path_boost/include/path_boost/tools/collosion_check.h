/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-31 20:41:03
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 21:42:00
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/tools/collosion_check.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef COLLOSION_CHECK_H_
#define COLLOSION_CHECK_H_

#include "map.h"
#include "car_model.hpp"
#include "path_boost/data_struct/basic_struct.hpp"

namespace PathBoostNS 
{
class CollisionChecker {
public:
    CollisionChecker() = delete;
    CollisionChecker(const grid_map::GridMap &in_gm);

    bool isSingleStateCollisionFreeImproved(const BState &current);

    bool isSingleStateCollisionFree(const BState &current);


private:
    const Map map_;
    CarModel car_;
};

}//namespace PathBoostNS

#endif