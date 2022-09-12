/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-03 19:24:24
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-03 19:29:37
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/data_struct/vehicle_state_frenet.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef VEHICLE_STATE_FRENET_HPP_
#define VEHICLE_STATE_FRENET_HPP_
#include <vector>

namespace PathBoostNS {

class BState;

class VehicleState {
 public:
    VehicleState();
    VehicleState(const BState &start_state,
                 const BState &end_state,
                 double offset = 0.0,
                 double heading_error = 0.0);
    ~VehicleState();
    const BState &getStartState() const;
    const BState &getTargetState() const;
    void setStartState(const BState &state);
    void setTargetState(const BState &state);
    std::vector<double> getInitError() const;
    void setInitError(double init_offset, double init_heading_error);

 private:
    // Initial state.
    BState *start_state_;
    // Target state.
    BState *target_state_;
    // Initial error with reference line.
    double initial_offset_{};
    double initial_heading_error_{};
};

}
#endif //VEHICLE_STATE_FRENET_HPP_
