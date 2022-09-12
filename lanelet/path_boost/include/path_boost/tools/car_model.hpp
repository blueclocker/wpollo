/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-30 22:39:24
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 20:38:34
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/tools/car_model.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CAR_MODEL_HPP_
#define CAR_MODEL_HPP_

#include "path_boost/data_struct/basic_struct.hpp"

namespace PathBoostNS
{
    
class CarModel
{
private:
    double width_;
    double length_;
    double front_length_;
    double back_length_;
    // Four corners:
    // f: front, r: rear
    // l: left, r: right
    BState fl_p_;//前左
    BState fr_p_;//前右
    BState rl_p_;//后左
    BState rr_p_;//后右
    Circle bounding_c_;//车身大圆
    std::vector<Circle> circles_;//六小圆
    void setCircles();
public:
    CarModel() = default;
    CarModel(double width, double front_length, double back_length);
    void init(double width, double front_length, double back_length);
    //六小圆在地图坐标系坐标及半径
    std::vector<Circle> getCircles(const BState &pos) const;
    //大圆在地图坐标系坐标及半径
    Circle getBoundingCircle(const BState &pos) const;
    ~CarModel();
};

}//namespace PathBoostNS
#endif