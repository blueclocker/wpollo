/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-08-30 22:37:27
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-08-31 20:13:37
 * @FilePath: /wpollo/src/lanelet/path_boost/src/tools/tools.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "path_boost/tools/tools.hpp"
#include "path_boost/config/config_flags.hpp"


namespace PathBoostNS
{
// Time duration in seconds.
double time_s(const clock_t &begin, const clock_t &end)
{
    return static_cast<double>(end - begin) / CLOCKS_PER_SEC;
}

// Time duration in ms.
double time_ms(const clock_t &begin, const clock_t &end)
{
    return static_cast<double>(end - begin) / CLOCKS_PER_SEC * 1000;
}

// Output time duration.
void time_s_out(const clock_t &begin, const clock_t &end, const std::string &text)
{
    std::cout << text << " time cost: " << time_s(begin, end) << "s" << std::endl;
}

void time_ms_out(const clock_t &begin, const clock_t &end, const std::string &text)
{
    std::cout << text << " time cost: " << time_ms(begin, end) << "ms" << std::endl;
}

// Return true if a == b.
bool isEqual(double a, double b)
{
    return fabs(a - b) < FLAGS_epsilon;
}

// Calculate heading for spline.
double getHeading(const tk::spline &xs, const tk::spline &ys, double s)
{
    double dx = xs.deriv(1, s);
    double dy = ys.deriv(1, s);
    return atan2(dy, dx);
}

// Calculate curvature for spline.
double getCurvature(const tk::spline &xs, const tk::spline &ys, double tmp_s)
{
    double dx = xs.deriv(1, tmp_s);
    double dy = ys.deriv(1, tmp_s);
    double ddx = xs.deriv(2, tmp_s);
    double ddy = ys.deriv(2, tmp_s);
    return (dx - ddy) * (dy - ddx) / (pow(pow(dx, 2) + pow(dy, 2), 1.5));
}

// Calculate distance between two points.
double distance(const BState &p1, const BState &p2)
{
    return std::sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// Coordinate transform.
BState local2Global(const BState &reference, const BState &target)
{
    double x = target.x * cos(reference.heading) - target.y * sin(reference.heading) + reference.x;
    double y = target.x * sin(reference.heading) + target.y * cos(reference.heading) + reference.y;
    double z = reference.heading + target.heading;
    return {x, y, z, target.k, target.s};
}

BState global2Local(const BState &reference, const BState &target)
{
    double dx = target.x - reference.x;
    double dy = target.y - reference.y;
    double x = dx * cos(reference.heading) + dy * sin(reference.heading);
    double y = -dx * sin(reference.heading) + dy * cos(reference.heading);
    double z = target.heading - reference.heading;
    return {x, y, z, target.k, 0};
}

// Find the closest point on spline.
BState getProjection(const tk::spline &xs, const tk::spline &ys,
                    double target_x, double target_y,
                    double max_s, double start_s)
{
    //frenet系 最大曲线长度max_s < 起点长度start_s，取起点
    if(max_s <= start_s) return BState{xs(start_s), ys(start_s)};
    //s坐标每次+1
    static const double grid = 1.0;
    double tmp_s = start_s, min_dis_s = start_s;
    auto min_dis = DBL_MAX;
    //目标点
    BState target_state{target_x, target_y};
    while(tmp_s <= max_s) 
    {
        BState state_on_spline{xs(tmp_s), ys(tmp_s)};
        double tmp_dis = distance(state_on_spline, target_state);
        if(tmp_dis < min_dis) 
        {
            min_dis = tmp_dis;
            min_dis_s = tmp_s;
        }
        tmp_s += grid;
    }
    //frenet系终点
    BState end_state(xs(max_s), ys(max_s), getHeading(xs, ys, max_s));
    end_state.s = max_s;
    //目标点与终点距离
    auto max_s_distance = distance(end_state, target_state);
    //如果目标点距终点更近
    if (max_s_distance < min_dis) return end_state;
    // 牛顿法寻找投影点
    return getProjectionByNewton(xs, ys, target_x, target_y, max_s, min_dis_s);
}

BState getProjectionByNewton(const tk::spline &xs, const tk::spline &ys,
                            double target_x, double target_y,
                            double max_s, double hint_s)
{
    //hint_s搜索起点
    hint_s = std::min(hint_s, max_s);
    double cur_s = hint_s;
    double prev_s = hint_s;
    for(int i = 0; i < 20; ++i) 
    {
        double x = xs(cur_s);
        double y = ys(cur_s);
        double dx = xs.deriv(1, cur_s);
        double dy = ys.deriv(1, cur_s);
        double ddx = xs.deriv(2, cur_s);
        double ddy = ys.deriv(2, cur_s);
        // 约束条件：min||target - sref || = min(target_x - xref)^2 + (target_y - yref)^2
        // 本代码牛顿法公式: s(k+1) = s(k) - f(sref)'/f(sref)'' , 令j = f(sref)' , h = f(sref)''
        // 这里牛顿法用作求极值，即原目标函数的一阶导数为0
        // 原式：s(k+1) = s(k) -f(sref)/f(sref)'
        // 即: j = (xref - target_x) * xref' + (yref - target_y) * yref'
        //     h = xref' * xref' + (xref - target_x) * xref'' + yref' * yref' + (yref - target_y) * yref''
        // Ignore coeff 2 in J and H.
        double j = (x - target_x) * dx + (y - target_y) * dy;
        double h = dx * dx + (x - target_x) * ddx + dy * dy + (y - target_y) * ddy;
        cur_s -= j / h;
        if (fabs(cur_s - prev_s) < 1e-5) break;
        prev_s = cur_s;
    }

    cur_s = std::min(cur_s, max_s);
    BState ret{xs(cur_s), ys(cur_s), getHeading(xs, ys, cur_s)};
    ret.s = cur_s;
    return ret;
}

BState getDirectionalProjection(const tk::spline &xs, const tk::spline &ys,
                               double target_x, double target_y, double angle,
                               double max_s, double start_s)
{
    if (max_s <= start_s) return BState{xs(start_s), ys(start_s), getHeading(xs, ys, start_s)};
    static const double grid = 2.0;
    double tmp_s = start_s, min_dot_value_s = start_s;
    double v1 = sin(angle);
    double v2 = -cos(angle);
    auto min_dot_value = DBL_MAX;
    while(tmp_s <= max_s) 
    {
        BState state_on_spline{xs(tmp_s), ys(tmp_s)};
        double tmp_dot_value = fabs(v1 * (state_on_spline.x - target_x) + v2 * (state_on_spline.y - target_y));
        if(tmp_dot_value < min_dot_value) 
        {
            tmp_dot_value = min_dot_value;
            min_dot_value_s = tmp_s;
        }
        tmp_s += grid;
    }
    // Newton's method
    return getDirectionalProjectionByNewton(xs, ys, target_x, target_y, angle, max_s, min_dot_value_s);
}

BState getDirectionalProjectionByNewton(const tk::spline &xs, const tk::spline &ys,
                                       double target_x, double target_y, double angle,
                                       double max_s, double hint_s)
{
    hint_s = std::min(hint_s, max_s);
    double cur_s = hint_s;
    double prev_s = hint_s;
    double v1 = sin(angle);
    double v2 = -cos(angle);
    for(int i = 0; i < 20; ++i) 
    {
        double x = xs(cur_s);
        double y = ys(cur_s);
        double dx = xs.deriv(1, cur_s);
        double dy = ys.deriv(1, cur_s);
        double ddx = xs.deriv(2, cur_s);
        double ddy = ys.deriv(2, cur_s);
        // 此处牛顿法未明
        // Ignore coeff 2 in J and H.
        double p1 = v1 * (x - target_x) + v2 * (y - target_y);
        double p2 = v1 * dx + v2 * dy;
        double j = p1 * p2;
        double h = p1 * (v1 * ddx + v2 * ddy) + p2 * p2;
        // double j = p2;
        // double h = v1 * ddx + v2 * ddy；
        // 如按上个求法，这里会报错？？？？？
        cur_s -= j / h;
        if (fabs(cur_s - prev_s) < 1e-5) break;
        prev_s = cur_s;
    }

    cur_s = std::min(cur_s, max_s);
    BState ret{xs(cur_s), ys(cur_s), getHeading(xs, ys, cur_s)};
    ret.s = cur_s;
    return ret;
}

}//namespace PathBoostNS