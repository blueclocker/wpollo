/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-30 22:11:18
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-03 20:06:04
 * @FilePath: /wpollo/src/lanelet/path_boost/include/path_boost/data_struct/data_struct.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef BASIC_STRUCT_HPP_
#define BASIC_STRUCT_HPP_

#include <iostream>
#include <vector>
#include <memory>
#include <cfloat>

namespace PathBoostNS{

struct BState
{
    //车二维姿态
    double x;
    double y;
    double heading;
    //曲率
    double k;
    double d_k;
    //里程、车速、加速度
    double s;
    double v;
    double a;
    BState() = default;
    BState(double x, double y, double heading = 0, double k = 0, double d_k = 0, 
           double s = 0, double v = 0, double a = 0):
           x(x), y(y), heading(heading), k(k), d_k(d_k), s(s), v(v), a(a){}
};

struct SLState : public BState
{
    SLState() = default;
    double l;
    double d_heading;
};

struct Circle
{
    double x;
    double y;
    double r;
    Circle() = default;
    Circle(double x, double y, double r): x(x), y(y), r(r){}
};

struct VehicleStateBound {
    VehicleStateBound() = default;
    struct SingleBound {
        SingleBound() = default;
        SingleBound &operator=(const std::vector<double> &bounds) {
            ub = bounds[0];
            lb = bounds[1];
        }
        void set(const std::vector<double> &bounds, const BState &center) {
            ub = bounds[0];
            lb = bounds[1];
            x = center.x;
            y = center.y;
            heading = center.heading;
        }
        double ub{}; // left
        double lb{}; // right
        double x{}, y{}, heading{};
    } front, rear, center;
};

// Point for A* search.
struct APoint {
    double x{};
    double y{};
    double s{};
    double l{};
    double g{};
    double h{};
    double dir{};
    // Layer denotes the index of the longitudinal layer that the point lies on.
    int layer{-1};
    int offset_idx{};
    double rough_upper_bound, rough_lower_bound;
    bool is_in_open_set{false};
    APoint *parent{nullptr};
    inline double f() {
        return g + h;
    }
};

// Point for DP.
struct DpPoint {
    double x, y, heading, s, l, cost = DBL_MAX, dir, dis_to_obs;
    int layer_index, lateral_index;
    double rough_upper_bound, rough_lower_bound;
    const DpPoint *parent = nullptr;
    bool is_feasible = true;
};

class PointComparator {
 public:
    bool operator()(APoint *a, APoint *b) {
        return a->f() > b->f();
    }
};

}//namespace PathBoostNS
#endif