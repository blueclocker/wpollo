/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-03 19:17:05
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-03 19:46:39
 * @FilePath: /wpollo/src/lanelet/path_boost/src/data_struct/reference_path.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "path_boost/data_struct/reference_path_impl.hpp"
#include "path_boost/data_struct/reference_path.hpp"
#include "path_boost/data_struct/basic_struct.hpp"
#include "path_boost/tools/spline.h"
#include "path_boost/tools/tools.hpp"
#include "path_boost/tools/map.h"

namespace PathBoostNS {

ReferencePath::ReferencePath() : reference_path_impl_(std::make_shared<ReferencePathImpl>()) {

}

const tk::spline &ReferencePath::getXS() const {
    return reference_path_impl_->getXS();
}

const tk::spline &ReferencePath::getYS() const {
    return reference_path_impl_->getYS();
}

double ReferencePath::getXS(double s) const {
    return reference_path_impl_->getXS()(s);
}

double ReferencePath::getYS(double s) const {
    return reference_path_impl_->getYS()(s);
}

void ReferencePath::clear() {
    reference_path_impl_->clear();
}

std::size_t ReferencePath::getSize() const {
    return reference_path_impl_->getSize();
}

double ReferencePath::getLength() const {
    return reference_path_impl_->getLength();
}

void ReferencePath::setLength(double s) {
    reference_path_impl_->setLength(s);
}

const std::vector<BState> &ReferencePath::getReferenceStates() const {
    return reference_path_impl_->getReferenceStates();
}

const std::vector<VehicleStateBound> &ReferencePath::getBounds() const {
    return reference_path_impl_->getBounds();
}

void ReferencePath::logBoundsInfo() const {
    reference_path_impl_->logBoundsInfo();
}

void ReferencePath::updateBounds(const Map &map) {
    reference_path_impl_->updateBoundsImproved(map);
}

bool ReferencePath::buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger) {
    return reference_path_impl_->buildReferenceFromSpline(delta_s_smaller, delta_s_larger);
}

bool ReferencePath::buildReferenceFromStates(const std::vector<PathBoostNS::BState> &states) {
    return reference_path_impl_->buildReferenceFromStates(states);
}

std::shared_ptr<VehicleStateBound> ReferencePath::isBlocked() const {
    return reference_path_impl_->isBlocked();
}

void ReferencePath::setSpline(const PathBoostNS::tk::spline &x_s,
                              const PathBoostNS::tk::spline &y_s,
                              double max_s) {
    reference_path_impl_->setSpline(x_s, y_s, max_s);
}

void ReferencePath::setOriginalSpline(const PathBoostNS::tk::spline &x_s,
                                      const PathBoostNS::tk::spline &y_s,
                                      double max_s) {
    reference_path_impl_->setOriginalSpline(x_s, y_s, max_s);
}

void ReferencePath::updateBoundsOnInputStates(const Map &map, std::vector<SLState> &input_sl_states) {
    reference_path_impl_->updateBoundsOnInputStates(map, input_sl_states);
}

}
