/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-30 22:38:31
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 20:39:11
 * @FilePath: /wpollo/src/lanelet/path_boost/src/tools/car_model.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "path_boost/tools/car_model.hpp"
#include "path_boost/tools/tools.hpp"

namespace PathBoostNS
{

CarModel::~CarModel()
{
}

CarModel::CarModel(double width, double front_length, double back_length):
    width_(width),
    length_(front_length + back_length),
    front_length_(front_length),
    back_length_(back_length),
    fl_p_(front_length, width / 2.0),
    fr_p_(front_length, -width / 2.0),
    rl_p_(-back_length, width / 2.0),
    rr_p_(-back_length, -width / 2.0)
{
    setCircles();
}

void CarModel::init(double width, double front_length, double back_length)
{
    width_ = width;
    length_ = back_length + front_length;
    front_length_ = front_length;
    back_length_ = back_length;
    fl_p_.x = front_length;
    fl_p_.y = width / 2.0;
    fr_p_.x = front_length;
    fr_p_.y = -width / 2.0;
    rl_p_.x = -back_length;
    rl_p_.y = width / 2.0;
    rr_p_.x = -back_length;
    rr_p_.y = -width / 2.0;
    setCircles();
}

void CarModel::setCircles()
{
    bounding_c_.x = (front_length_ - back_length_) / 2.0;
    bounding_c_.y = 0;
    bounding_c_.r = sqrt(pow(length_ / 2, 2) + pow(width_ / 2, 2));
    double small_circle_shift = width_ / 4.0;
    double small_circle_radius = sqrt(2 * pow(small_circle_shift, 2));
    double large_circle_radius = sqrt(pow(width_, 2) + pow((length_ - width_) / 2.0, 2)) / 2;
    // rr
    circles_.emplace_back(rr_p_.x + small_circle_shift, rr_p_.y + small_circle_shift, small_circle_radius);
    // rl
    circles_.emplace_back(rl_p_.x + small_circle_shift, rl_p_.y - small_circle_shift, small_circle_radius);
    // fr
    circles_.emplace_back(fr_p_.x - small_circle_shift, fr_p_.y + small_circle_shift, small_circle_radius);
    // fl
    circles_.emplace_back(fl_p_.x - small_circle_shift, fl_p_.y - small_circle_shift, small_circle_radius);
    // fm
    circles_.emplace_back(bounding_c_.x + (length_ - width_) / 4, 0, large_circle_radius);
    // rm
    circles_.emplace_back(bounding_c_.x - (length_ - width_) / 4, 0, large_circle_radius);
}

std::vector<Circle> CarModel::getCircles(const BState &pos) const
{
    //pos车位置，state车辆坐标系圆心坐标
    std::vector<Circle> result;
    for (const auto &circle : circles_) {
        BState state(circle.x, circle.y);
        auto global_state = local2Global(pos, state);
        result.emplace_back(global_state.x, global_state.y, circle.r);
    }
    return result;
}

Circle CarModel::getBoundingCircle(const BState &pos) const
{
    //pos车位置，state大圆在车辆坐标系坐标
    BState state(bounding_c_.x, bounding_c_.y);
    auto global_state = local2Global(pos, state);
    return {global_state.x, global_state.y, bounding_c_.r};
}

}//namespace PathBoostNS



