/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-03 19:24:07
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-03 20:04:17
 * @FilePath: /wpollo/src/lanelet/path_boost/src/data_struct/vehicle_state_frenet.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "path_boost/data_struct/vehicle_state_frenet.hpp"
#include "path_boost/data_struct/basic_struct.hpp"
namespace PathBoostNS {

VehicleState::VehicleState() :
    start_state_(new BState),
    target_state_(new BState),
    initial_offset_(0),
    initial_heading_error_(0) {}

VehicleState::VehicleState(const PathBoostNS::BState &start_state,
                           const PathBoostNS::BState &end_state,
                           double offset,
                           double heading_error) :
    start_state_(new BState{start_state}),
    target_state_(new BState{end_state}),
    initial_offset_(offset),
    initial_heading_error_(heading_error) {}

VehicleState::~VehicleState() {
    delete start_state_;
    delete target_state_;
}

const BState& VehicleState::getStartState() const {
    return *start_state_;
}

const BState& VehicleState::getTargetState() const {
    return *target_state_;
}

void VehicleState::setStartState(const PathBoostNS::BState &state) {
    *start_state_ = state;
}

void VehicleState::setTargetState(const PathBoostNS::BState &state) {
    *target_state_ = state;
}

std::vector<double> VehicleState::getInitError() const {
    return {initial_offset_, initial_heading_error_};
}

void VehicleState::setInitError(double init_offset, double init_heading_error) {
    initial_offset_ = init_offset;
    initial_heading_error_ = init_heading_error;
}

}