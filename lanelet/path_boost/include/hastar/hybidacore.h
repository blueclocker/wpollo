/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-06 22:03:28
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @LastEditTime: 2022-09-12 20:28:23
 * @FilePath: /wpollo/src/lanelet/path_boost/include/hastar/hybidacore.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef HYBIDACORE_H
#define HYBIDACORE_H_

#include <queue>
#include "rs_path.h"
#include "state.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <map>
#include <chrono>

namespace HybidA
{
class hybidacore
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // nav_msgs::OccupancyGridPtr mapptr;
    uint8_t *mapptr = nullptr;
    // HAstate *start;
    // HAstate *goal;
    double wheel_base_; //The distance between the front and rear axles
    double segment_length_;
    double move_step_size_;
    double steering_radian_step_size_;
    double steering_radian_; //radian
    double tie_breaker_;

    double shot_distance_;
    int segment_length_discrete_num_;
    int steering_discrete_num_;
    double steering_penalty_;
    double reversing_penalty_;
    double steering_change_penalty_;

    double path_length_ = 0.0;
    std::shared_ptr<RSPath> rs_path_ptr_;
    VecXd vehicle_shape_;
    MatXd vehicle_shape_discrete_;
    HAstate::ptr terminal_node_ptr_ = nullptr;
    HAstate::ptr ***state_node_map_ = nullptr;//采样空间
    std::multimap<double, HAstate::ptr> openlist_;
    // std::priority_queue<HAstate::ptr> openlist_;

    double STATE_GRID_RESOLUTION_{}, MAP_GRID_RESOLUTION_{};
    double ANGULAR_RESOLUTION_{};
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{}, STATE_GRID_SIZE_PHI_{};
    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{};

    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{};

    bool beyondBoundary(const Vec2d &pt) const;
    static double Mod2Pi(const double &x);
    bool hasObstacle(int grid_index_x, int grid_index_y) const;
    Vec3i state2Index(const Vec3d &state) const;
    Vec2d mapGridIndex2Coordinate(const Vec2i &grid_index) const;
    Vec2i coordinate2MapGridIndex(const Vec2d &pt) const;
    bool analyticExpansions(const HAstate::ptr &current_node_ptr, const HAstate::ptr &goal_node_ptr, double &length);
    void getNeighbors(const HAstate::ptr &current_node_ptr, std::vector<HAstate::ptr> &neighbor_nodes);
    double computeG(const HAstate::ptr &current_node_ptr, const HAstate::ptr &neighbor_node_ptr) const;
    double computeH(const HAstate::ptr &current_node_ptr, const HAstate::ptr &terminal_node_ptr) const;
    bool lineCheck(double x0, double y0, double x1, double y1);
    bool collosionCheck(const double &x, const double &y, const double &theta);
    void dynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta) const;
public:
    hybidacore() = delete;
    hybidacore(double steering_angle, int steering_angle_discrete_num, double segment_length,
                int segment_length_discrete_num, double wheel_base, double steering_penalty,
                double reversing_penalty, double steering_change_penalty, double shot_distance,
                int grid_size_phi = 72);
    ~hybidacore();
    void initptr(const nav_msgs::OccupancyGridConstPtr &current_costmap_ptr_, double map_grid_resolution);
    void init(const nav_msgs::OccupancyGrid &current_costmap_ptr_, double map_grid_resolution);
    bool search(const Vec3d &start, const Vec3d &goal);
    VectorVec3d getPath() const;
    VectorVec4d getSearchedTree();
    void reset();
    void setVehicleShape(double length, double width, double rear_axle_dist);
};

}

#endif