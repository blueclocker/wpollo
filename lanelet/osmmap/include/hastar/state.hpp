/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-06 21:42:32
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-12 14:14:49
 * @FilePath: /catkin_ws/src/hastar/include/hastar/state.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef STATE_HPP_
#define STATE_HPP_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "type.hpp"

namespace HybidA
{
struct HAstate
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum NODESTAUS
    {
        NOT_VISITED = 0,
        OPENLIST = 1,
        CLOSELIST = 2
    };

    enum DIRECTION
    {
        FORWORD = 0,
        BACKWORD = 1,
        NO = 2
    };
    HAstate() = delete;
    // ~HAstate()
    // {
    //     std::cout << "~hastate" << std::endl;
    // }
    explicit HAstate(const Vec3i& grid_index_)
    {
        grid_index = grid_index_;
        parent = nullptr;
        node_statue = NOT_VISITED;
    }

    void reset()
    {
        parent = nullptr;
        node_statue = NOT_VISITED;
    }
    
    NODESTAUS node_statue;
    DIRECTION direction{};
    Vec3d pos;
    Vec3i grid_index;
    HAstate *parent;
    double g = 0;
    double f = 0;
    int steering_grade_ = 0;
    VectorVec3d intermediate_states_;
    typedef HAstate* ptr;

    // friend bool operator < (const HAstate &a, const HAstate &b)
    // {    
    //     return a.f > b.f;    //f最小值优先
    // }
};



}//namespace hybida
#endif
