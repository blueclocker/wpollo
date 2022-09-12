/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-03 19:17:21
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-03 19:58:26
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/data_struct/reference_path.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef REFERENCE_PATH_HPP_
#define REFERENCE_PATH_HPP_
#include <memory>
#include <vector>
#include <tuple>

namespace PathBoostNS {
class Map;
class Config;
class BState;
class VehicleStateBound;
namespace tk {
class spline;
}
class ReferencePathImpl;

class ReferencePath {
 public:
    ReferencePath();
    const tk::spline &getXS() const;
    const tk::spline &getYS() const;
    double getXS(double s) const;
    double getYS(double s) const;
    void setSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    void setOriginalSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    void clear();
    std::size_t getSize() const;
    double getLength() const;
    void setLength(double s);
    const std::vector<BState> &getReferenceStates() const;
    const std::vector<VehicleStateBound> &getBounds() const;
    void logBoundsInfo() const;
    // Calculate upper and lower bounds for each covering circle.
    void updateBounds(const Map &map);
    // Calculate reference_states_ from x_s_ and y_s_, given delta s.
    bool buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger);
    bool buildReferenceFromStates(const std::vector<PathBoostNS::BState> &states);
    std::shared_ptr<VehicleStateBound> isBlocked() const;
    void updateBoundsOnInputStates(const Map &map, std::vector<SLState> &input_sl_states);
 private:
    std::shared_ptr<ReferencePathImpl> reference_path_impl_;
};
}
#endif //REFERENCE_PATH_HPP_
