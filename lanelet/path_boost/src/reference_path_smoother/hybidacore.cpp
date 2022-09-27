/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-12 13:58:09
 * @LastEditTime: 2022-09-12 20:28:06
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/path_boost/src/reference_path_smoother/hybidacore.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "hastar/hybidacore.h"


namespace HybidA
{
hybidacore::hybidacore(double steering_angle, int steering_angle_discrete_num, double segment_length,
                int segment_length_discrete_num, double wheel_base, double steering_penalty,
                double reversing_penalty, double steering_change_penalty, double shot_distance,
                int grid_size_phi)
{
    wheel_base_ = wheel_base;
    segment_length_ = segment_length;
    steering_radian_ = steering_angle * M_PI / 180.0; // angle to radian
    steering_discrete_num_ = steering_angle_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;
    move_step_size_ = segment_length / segment_length_discrete_num;
    segment_length_discrete_num_ = segment_length_discrete_num;
    // segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);
    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;
    shot_distance_ = shot_distance;

    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));
    tie_breaker_ = 1.0 + 1e-3;

    STATE_GRID_SIZE_PHI_ = grid_size_phi;
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0;
}

hybidacore::~hybidacore()
{
    if (mapptr != nullptr) {
        delete[] mapptr;
        mapptr = nullptr;
    }
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }

                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }

            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    terminal_node_ptr_ = nullptr;
}

void hybidacore::initptr(const nav_msgs::OccupancyGridConstPtr &current_costmap_ptr_, double map_grid_resolution)
{
    setVehicleShape(4.7, 2.0, 1.3);

    // map_x_lower_ = x_lower;
    // map_x_upper_ = x_upper;
    // map_y_lower_ = y_lower;
    // map_y_upper_ = y_upper;
    // STATE_GRID_RESOLUTION_ = state_grid_resolution;
    // MAP_GRID_RESOLUTION_ = map_grid_resolution;

    map_x_lower_ = current_costmap_ptr_->info.origin.position.x;
    map_x_upper_ = 1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution;
    map_y_lower_ = current_costmap_ptr_->info.origin.position.y;
    map_y_upper_ = 1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution;
    STATE_GRID_RESOLUTION_ = current_costmap_ptr_->info.resolution;
    MAP_GRID_RESOLUTION_ = map_grid_resolution;

    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);

    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

    if (mapptr) {
        delete[] mapptr;
        mapptr = nullptr;
    }

    mapptr = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_];

    //初始化，对输入的栅格地图更改分辨率为map_grid_resolution, (w, h)为新地图坐标
    unsigned int map_w = std::floor(current_costmap_ptr_->info.width / map_grid_resolution);
    unsigned int map_h = std::floor(current_costmap_ptr_->info.height / map_grid_resolution);
    std::cout << "map_w: " << map_w << ", map_h: " << map_h << std::endl;
    for (unsigned int w = 0; w < map_w; ++w) 
    {
        for (unsigned int h = 0; h < map_h; ++h) 
        {
            auto x = static_cast<unsigned int> ((w + 0.5) * map_grid_resolution
                                                / current_costmap_ptr_->info.resolution);
            auto y = static_cast<unsigned int> ((h + 0.5) * map_grid_resolution
                                                / current_costmap_ptr_->info.resolution);

            if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) 
            {
                if (w < 0u || w > static_cast<unsigned int>(MAP_GRID_SIZE_X_)
                     || h < 0u || h > static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) 
                {
                    continue;
                }

                mapptr[w + h * MAP_GRID_SIZE_X_] = 1;
                // static int cnt = 0;
                // cnt++;
                // std::cout << "set obstacle" << cnt << std::endl;
            }
                // std::cout << mapptr[x + y * MAP_GRID_SIZE_X_] << " " ;
        }
        // std::cout << std::endl;
    }


    if (state_node_map_) 
    {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) 
        {

            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) 
            {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) 
                {
                    if (state_node_map_[i][j][k] != nullptr) 
                    {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    state_node_map_ = new HAstate::ptr **[STATE_GRID_SIZE_X_];
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) 
    {
        state_node_map_[i] = new HAstate::ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) 
        {
            state_node_map_[i][j] = new HAstate::ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) 
            {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}

void hybidacore::init(const nav_msgs::OccupancyGrid &current_costmap_ptr_, double map_grid_resolution)
{
    setVehicleShape(4.7, 2.0, 1.3);

    map_x_lower_ = current_costmap_ptr_.info.origin.position.x;
    map_x_upper_ = 1.0 * current_costmap_ptr_.info.width * current_costmap_ptr_.info.resolution;
    map_y_lower_ = current_costmap_ptr_.info.origin.position.y;
    map_y_upper_ = 1.0 * current_costmap_ptr_.info.height * current_costmap_ptr_.info.resolution;
    STATE_GRID_RESOLUTION_ = current_costmap_ptr_.info.resolution;
    MAP_GRID_RESOLUTION_ = map_grid_resolution;

    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);

    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

    if (mapptr) {
        delete[] mapptr;
        mapptr = nullptr;
    }

    mapptr = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_];

    //初始化，对输入的栅格地图更改分辨率为map_grid_resolution, (w, h)为新地图坐标
    unsigned int map_w = std::floor(current_costmap_ptr_.info.width / map_grid_resolution);
    unsigned int map_h = std::floor(current_costmap_ptr_.info.height / map_grid_resolution);
    std::cout << "map_w: " << map_w << ", map_h: " << map_h << std::endl;
    for (unsigned int w = 0; w < map_w; ++w) 
    {
        for (unsigned int h = 0; h < map_h; ++h) 
        {
            auto x = static_cast<unsigned int> ((w + 0.5) * map_grid_resolution
                                                / current_costmap_ptr_.info.resolution);
            auto y = static_cast<unsigned int> ((h + 0.5) * map_grid_resolution
                                                / current_costmap_ptr_.info.resolution);

            if (current_costmap_ptr_.data[y * current_costmap_ptr_.info.width + x]) 
            {
                if (w < 0u || w > static_cast<unsigned int>(MAP_GRID_SIZE_X_)
                     || h < 0u || h > static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) 
                {
                    continue;
                }

                mapptr[w + h * MAP_GRID_SIZE_X_] = 1;
            }
        }
    }


    if (state_node_map_) 
    {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) 
        {

            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) 
            {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) 
                {
                    if (state_node_map_[i][j][k] != nullptr) 
                    {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    state_node_map_ = new HAstate::ptr **[STATE_GRID_SIZE_X_];
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) 
    {
        state_node_map_[i] = new HAstate::ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) 
        {
            state_node_map_[i][j] = new HAstate::ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) 
            {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}

bool hybidacore::search(const Vec3d &start, const Vec3d &goal)
{
    std::cout << "searching" << std::endl;
    std::chrono::time_point<std::chrono::system_clock> starttime, endtime;
    starttime = std::chrono::system_clock::now();
    const Vec3i start_grid_index = state2Index(start);
    const Vec3i goal_grid_index = state2Index(goal);
    
    auto goal_node_ptr = new HAstate(goal_grid_index);
    goal_node_ptr->pos = goal;
    goal_node_ptr->direction = HAstate::DIRECTION::NO;
    goal_node_ptr->steering_grade_ = 0;

    auto start_node_ptr = new HAstate(start_grid_index);
    start_node_ptr->pos = start;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction = HAstate::DIRECTION::NO;
    start_node_ptr->node_statue = HAstate::NODESTAUS::OPENLIST;
    start_node_ptr->intermediate_states_.emplace_back(start);
    start_node_ptr->g = 0.0;
    start_node_ptr->f = computeH(start_node_ptr, goal_node_ptr);

    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    // while(!openlist_.empty()) openlist_.pop();
    // openlist_.push(start_node_ptr);
    openlist_.clear();
    openlist_.insert(std::make_pair(0, start_node_ptr));

    std::vector<HAstate::ptr> neighbor_nodes_ptr;
    HAstate::ptr current_node_ptr;
    HAstate::ptr neighbor_node_ptr;

    int count = 0;
    while(!openlist_.empty())
    {
        // std::cout << "openlist size is: " << openlist_.size() << std::endl;
        // current_node_ptr = openlist_.top();
        // current_node_ptr->node_statue = CLOSELIST;
        // std::cout << "openlist fist f is: " << current_node_ptr->f << std::endl;
        // openlist_.pop();
        current_node_ptr = openlist_.begin()->second;
        current_node_ptr->node_statue = HAstate::NODESTAUS::CLOSELIST;
        openlist_.erase(openlist_.begin());

        if ((current_node_ptr->pos.head(2) - goal_node_ptr->pos.head(2)).norm() <= shot_distance_) 
        {
            double rs_length = 0.0;
            if (analyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) 
            {
                terminal_node_ptr_ = goal_node_ptr;

                HAstate::ptr grid_node_ptr = terminal_node_ptr_->parent;
                while (grid_node_ptr != nullptr) 
                {
                    grid_node_ptr = grid_node_ptr->parent;
                    path_length_ = path_length_ + segment_length_;
                }
                path_length_ = path_length_ - segment_length_ + rs_length;

                endtime = std::chrono::system_clock::now();
                std::chrono::duration<double> use_time = endtime - starttime;
                std::cout << "use time(ms): " << use_time.count() * 1000 << std::endl;
                std::cout << "RS shot successful!" << std::endl;

                return true;
            }
        }

        getNeighbors(current_node_ptr, neighbor_nodes_ptr);
        // std::cout << "neighbor_nodes_ptr size is " << neighbor_nodes_ptr.size() << std::endl << std::endl;

        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) 
        {
            neighbor_node_ptr = neighbor_nodes_ptr[i];
            // std::cout << "neighbor_node: " << neighbor_node_ptr->grid_index.x() << " " << neighbor_node_ptr->grid_index.y() << std::endl;

            const double neighbor_edge_cost = computeG(current_node_ptr, neighbor_node_ptr);
            const double current_h = computeH(current_node_ptr, goal_node_ptr) * tie_breaker_;

            const Vec3i &index = neighbor_node_ptr->grid_index;
            // std::cout << STATE_GRID_SIZE_X_ << " " << STATE_GRID_SIZE_Y_ << " " << STATE_GRID_SIZE_PHI_ << std::endl;
            // std::cout << "index x, y, z: " << index.x() << " " << index.y() << " " << index.z() << std::endl;
            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) 
            {
                neighbor_node_ptr->g = current_node_ptr->g + neighbor_edge_cost;
                neighbor_node_ptr->parent = current_node_ptr;
                neighbor_node_ptr->node_statue = HAstate::NODESTAUS::OPENLIST;
                neighbor_node_ptr->f = neighbor_node_ptr->g + current_h;
                // openlist_.push(neighbor_node_ptr);
                openlist_.insert(std::make_pair(neighbor_node_ptr->f, neighbor_node_ptr));
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_statue == HAstate::NODESTAUS::OPENLIST) {
                double g_cost_temp = current_node_ptr->g + neighbor_edge_cost;

                if (state_node_map_[index.x()][index.y()][index.z()]->g > g_cost_temp) {
                    neighbor_node_ptr->g = g_cost_temp;
                    neighbor_node_ptr->f = g_cost_temp + current_h;
                    neighbor_node_ptr->parent = current_node_ptr;
                    neighbor_node_ptr->node_statue = HAstate::NODESTAUS::CLOSELIST;

                    delete state_node_map_[index.x()][index.y()][index.z()];
                    state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                } else {
                    delete neighbor_node_ptr;
                }
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_statue == HAstate::NODESTAUS::CLOSELIST) {
                delete neighbor_node_ptr;
                continue;
            }
        }
        
        count++;
        if (count > 50000) {
            endtime = std::chrono::system_clock::now();
            std::chrono::duration<double> use_time = endtime - starttime;
            std::cout << "use time(ms): " << use_time.count() * 1000 << std::endl;
            std::cout << "Exceeded the number of iterations, the search failed" << std::endl;
            return false;
        }
    }

    return false;
}

void hybidacore::reset() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }

    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}

VectorVec3d hybidacore::getPath() const
{
    VectorVec3d path;

    std::vector<HAstate::ptr> temp_nodes;

    HAstate::ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent;
    }

    std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(),
                    node->intermediate_states_.end());
    }

    return path;
}

VectorVec4d hybidacore::getSearchedTree() 
{
    VectorVec4d tree;
    Vec4d point_pair;

    // visited_node_number_ = 0;
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k]->parent == nullptr) {
                    continue;
                }

                const unsigned int number_states = state_node_map_[i][j][k]->intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < number_states; ++l) {
                    point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[l].head(2);
                    point_pair.tail(2) = state_node_map_[i][j][k]->intermediate_states_[l + 1].head(2);

                    tree.emplace_back(point_pair);
                }

                point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[0].head(2);
                point_pair.tail(2) = state_node_map_[i][j][k]->parent->pos.head(2);
                tree.emplace_back(point_pair);
                // visited_node_number_++;
            }
        }
    }

    return tree;
}

void hybidacore::setVehicleShape(double length, double width, double rear_axle_dist)
{
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);

    const double step_size = move_step_size_;
    const auto N_length = static_cast<unsigned int>(length / step_size);
    const auto N_width = static_cast<unsigned int> (width / step_size);
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

    const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                     - vehicle_shape_.block<2, 1>(0, 0)).normalized();
    for (unsigned int i = 0; i < N_length; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i)
                = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                     - vehicle_shape_.block<2, 1>(2, 0)).normalized();
    for (unsigned int i = 0; i < N_width; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}

double hybidacore::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

bool hybidacore::hasObstacle(int grid_index_x, int grid_index_y) const
{
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (mapptr[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

bool hybidacore::beyondBoundary(const Vec2d &pt) const
{
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_ || 
           pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}

Vec3i hybidacore::state2Index(const Vec3d &state) const
{
    Vec3i index;

    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);

    return index;
}

Vec2d hybidacore::mapGridIndex2Coordinate(const Vec2i &grid_index) const
{
    Vec2d pt;
    pt.x() = ((double) grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
    pt.y() = ((double) grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;

    return pt;
}

Vec2i hybidacore::coordinate2MapGridIndex(const Vec2d &pt) const {
    Vec2i grid_index;

    grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);
    return grid_index;
}

bool hybidacore::analyticExpansions(const HAstate::ptr &current_node_ptr, const HAstate::ptr &goal_node_ptr, double &length)
{
    VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->pos,
                                                        goal_node_ptr->pos,
                                                        move_step_size_, length);

    for (const auto &pose: rs_path_poses)
        if (beyondBoundary(pose.head(2)) || !collosionCheck(pose.x(), pose.y(), pose.z())) 
        {
            return false;
        };

    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);

    return true;
}

void hybidacore::getNeighbors(const HAstate::ptr &current_node_ptr, std::vector<HAstate::ptr> &neighbor_nodes)
{
    // std::cout << "getNeighbors ing" << std::endl;
    neighbor_nodes.clear();

    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i)
    {
        VectorVec3d intermediate_state;
        bool has_obstacle = false;

        double x = current_node_ptr->pos.x();
        double y = current_node_ptr->pos.y();
        double theta = current_node_ptr->pos.z();

        const double phi = i * steering_radian_step_size_;
        // std::cout << "get current pos" << std::endl;

        // forward
        for (int j = 1; j <= segment_length_discrete_num_; j++) 
        {
            // std::cout << "before forword dynamicModel" << std::endl;
            dynamicModel(move_step_size_, phi, x, y, theta);
            // std::cout << "after forword dynamicModel" << std::endl;
            intermediate_state.emplace_back(Vec3d(x, y, theta));//??????

            // std::cout << "before forword collosionCheck" << std::endl;
            if (!collosionCheck(x, y, theta)) 
            {
                has_obstacle = true;
                // std::cout << "forword has obstacle" << std::endl;
                break;
            }
            // std::cout << "after forword collosionCheck" << std::endl;
        }
        // std::cout << "get new forword point" << std::endl;

        Vec3i grid_index = state2Index(intermediate_state.back());
        if (!beyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) 
        {
            // std::cout << "after forword state2Index" << std::endl;
            HAstate::ptr neighbor_forward_node_ptr = new HAstate(grid_index);
            // std::cout << "after forword new HAstate" << std::endl;//
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_forward_node_ptr->pos = intermediate_state.back();
            neighbor_forward_node_ptr->steering_grade_ = i;
            neighbor_forward_node_ptr->direction = HAstate::DIRECTION::FORWORD;
            neighbor_nodes.push_back(neighbor_forward_node_ptr);
        }
        // std::cout << "get forword end, i = " << i << std::endl;

        // backward
        has_obstacle = false;
        intermediate_state.clear();
        x = current_node_ptr->pos.x();
        y = current_node_ptr->pos.y();
        theta = current_node_ptr->pos.z();
        for (int j = 1; j <= segment_length_discrete_num_; j++) 
        {
            dynamicModel(-move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!collosionCheck(x, y, theta)) 
            {
                has_obstacle = true;
                // std::cout << "backword has obstacle" << std::endl;
                break;
            }
        }
        // std::cout << "get new backword point, intermediate_state size is " << intermediate_state.size() << std::endl;

        if (!beyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) 
        {
            grid_index = state2Index(intermediate_state.back());
            // std::cout << "after backword state2Index" << std::endl;//
            HAstate::ptr neighbor_backward_node_ptr = new HAstate(grid_index);//???????
            // std::cout << "after backword new HAstate" << std::endl;//
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->pos = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction = HAstate::DIRECTION::BACKWORD;
            // std::cout << "before backword push_back" << std::endl;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
        }
        // std::cout << "get backword end, i = " << i << std::endl;
    }
    // std::cout << "getNeighbors !!end!!" << std::endl;
}

double hybidacore::computeG(const HAstate::ptr &current_node_ptr, const HAstate::ptr &neighbor_node_ptr) const
{
    double g;
    if(neighbor_node_ptr->direction == HAstate::DIRECTION::FORWORD)
    {
        //前进
        if(current_node_ptr->steering_grade_ == neighbor_node_ptr->steering_grade_)
        {
            //转向与上一次相同
            if(current_node_ptr->steering_grade_ == 0)
            {
                //不转向
                g = segment_length_;
            }else{
                //转向
                g = segment_length_ * steering_penalty_;
            }
        }else{
            //转向不同
            if(current_node_ptr->steering_grade_ == 0)
            {
                g = segment_length_ * steering_change_penalty_;
            }else{
                g = segment_length_ * steering_penalty_ * steering_change_penalty_;
            }
        }
    }else{
        //后退
        if(current_node_ptr->steering_grade_ == neighbor_node_ptr->steering_grade_)
        {
            if(current_node_ptr->steering_grade_ == 0)
            {
                g = segment_length_ * reversing_penalty_;
            }else{
                g = segment_length_ * reversing_penalty_ * steering_penalty_;
            }
        }else{
            if(current_node_ptr->steering_grade_ == 0)
            {
                g = segment_length_ * reversing_penalty_ * steering_change_penalty_;
            }else{
                g = segment_length_ * reversing_penalty_ * steering_change_penalty_ * steering_penalty_;
            }
        }
    }
    return g;
}

double hybidacore::computeH(const HAstate::ptr &current_node_ptr, const HAstate::ptr &terminal_node_ptr) const
{
    //current_node与terminal_node的曼哈顿距离
    double h = (current_node_ptr->pos.head(2) - terminal_node_ptr->pos.head(2)).lpNorm<1>();//1范数

    //到终点的启发距离小于阈值时，采用RS曲线长度替代曼哈顿距离
    if(h < 3.0 * shot_distance_)
    {
        h = rs_path_ptr_->Distance(current_node_ptr->pos.x(), current_node_ptr->pos.y(), current_node_ptr->pos.z(),
                                   terminal_node_ptr->pos.x(), terminal_node_ptr->pos.y(), terminal_node_ptr->pos.z());
    }
    return h;
}

bool hybidacore::lineCheck(double x0, double y0, double x1, double y1)
{
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

    if (steep) 
    {
        std::swap(x0, y0);
        std::swap(y1, x1);
    }

    if (x0 > x1) 
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;
    decltype(delta_x) error = 0;
    decltype(delta_x) y_step;
    auto yk = y0;

    if (y0 < y1) 
    {
        y_step = 1;
    } else {
        y_step = -1;
    }

    auto N = static_cast<unsigned int>(x1 - x0);
    for (unsigned int i = 0; i < N; ++i) 
    {
        if (steep) 
        {
            if (hasObstacle(yk, x0 + i * 1.0)
                || beyondBoundary(Vec2d(yk * MAP_GRID_RESOLUTION_, (x0 + i) * MAP_GRID_RESOLUTION_))) 
            {
                return false;
            }
        } else {
            if (hasObstacle(x0 + i * 1.0, yk)
                || beyondBoundary(Vec2d((x0 + i) * MAP_GRID_RESOLUTION_, yk * MAP_GRID_RESOLUTION_))) 
            {
                return false;
            }
        }

        error += delta_error;
        if (error >= 0.5) 
        {
            yk += y_step;
            error = error - 1.0;
        }
    }

    return true;
}

bool hybidacore::collosionCheck(const double &x, const double &y, const double &theta)
{
    Mat2d R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta), std::cos(theta);

    MatXd transformed_vehicle_shape;
    transformed_vehicle_shape.resize(8, 1);
    for (unsigned int i = 0; i < 4u; ++i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0)
                = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
    }

    Vec2i transformed_pt_index_0 = coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(0, 0)
    );
    Vec2i transformed_pt_index_1 = coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(2, 0)
    );

    Vec2i transformed_pt_index_2 = coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(4, 0)
    );

    Vec2i transformed_pt_index_3 = coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(6, 0)
    );

    double y1, y0, x1, x0;
    // pt1 -> pt0
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!lineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt2 -> pt1
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!lineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt3 -> pt2
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!lineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt0 -> pt3
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!lineCheck(x0, y0, x1, y1)) {
        return false;
    }

    // num_check_collision++;
    return true;
}

void hybidacore::dynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta) const
{
    //(x,y, yaw)是车辆的当前姿态；
    //distance是车辆在当前行驶方向上前进的距离；
    //steer是方向盘与车辆行驶方向的夹角；
    //函数返回的是满足车辆运动学约束的下一个姿态点。
    //x += distance * cos(yaw)
    //y += distance * sin(yaw)
    //yaw += pi_2_pi(distance * tan(steer) / L)  # distance/2
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}


}