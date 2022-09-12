/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-31 15:07:06
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-03 20:33:46
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/pathboost_core.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PATHBOOST_CORE_H_
#define PATHBOOST_CORE_H_

#include <string>
#include <vector>
#include <memory>
#include "grid_map_core/grid_map_core.hpp"
//#include <tuple>
//#include <glog/logging.h>

namespace PathBoostNS {

class ReferencePath;
class BState;
class SLState;
class Map;
class CollisionChecker;
class VehicleState;

class PathBoost {
public:
    PathBoost() = delete;
    PathBoost(const BState &start_state,
              const BState &end_state,
              const grid_map::GridMap &map);
    PathBoost(const PathBoost &boost) = delete;
    PathBoost &operator=(const PathBoost &boost) = delete;

    // Call this to get the boost path.
    bool solve(const std::vector<BState> &reference_points, std::vector<SLState> *final_path);

    // Only for visualization purpose.
//    std::vector<std::tuple<State, double, double>> display_abnormal_bounds() const;
    const ReferencePath &getReferencePath() const;

private:
    // Core function.
    bool boostPath(std::vector<SLState> *final_path);

    // Divide smoothed path into segments.
    bool processReferencePath();

    // Calculate init error between the vehicle and the ref.
    void processInitState();

    //
    void setReferencePathLength();

    std::shared_ptr<Map> grid_map_;
    std::shared_ptr<CollisionChecker> collision_checker_;
    std::shared_ptr<ReferencePath> reference_path_;
    std::shared_ptr<VehicleState> vehicle_state_;
};
}

#endif //PATHBOOST_CORE_H_
