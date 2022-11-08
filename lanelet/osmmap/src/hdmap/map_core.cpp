/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-24 15:16:06
 * @LastEditTime: 2022-11-03 15:13:39
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/hdmap/map_core.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "osmmap/map_core.h"

namespace map
{
HDMap::HDMap(ros::NodeHandle n):n_(n)
{
    std::string file_path_, file_name_;
    double origin_lat_, origin_lon_, origin_ele_;
    use_gnss_ = true;
    n_.getParam("file_path", file_path_);
    n_.getParam("file_name", file_name_);
    n_.getParam("origin_lat", origin_lat_);
    n_.getParam("origin_lon", origin_lon_);
    n_.getParam("origin_ele", origin_ele_);
    n_.getParam("use_gnss", use_gnss_);

    //可视化
    map_pub_ = n_.advertise<visualization_msgs::MarkerArray>("map", 1);
    path_pub_ = n_.advertise<visualization_msgs::MarkerArray>("path", 1);
    gpspath_pub_ = n_.advertise<nav_msgs::Path>("gpspath_info", 1);
    // gridmap_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("gridmap_info", 1, true);
    //导航信息
    navigation_pub_ = n_.advertise<osmmap::Navigation>("navigation_info", 1);
    carstate_pub_ = n_.advertise<osmmap::CarState>("carstate_info", 1);
    lanes_pub_ = n_.advertise<osmmap::Lanes>("lanes_info", 1);
    //全局路径(平滑后)，只在gps_callback调用
    golbalpath_pub_ = n_.advertise<visualization_msgs::Marker>("golbalpath_info", 1);
    //车辆当前状态
    speed_pub_ = n_.advertise<std_msgs::Float32>("car_speed", 1);

    std::cout << "---------------------------------------------------" << std::endl;
    vectormap_ = new Map(file_path_, file_name_);
    vectormap_->SetOrigin(origin_lat_, origin_lon_, origin_ele_);
    std::cout << "************** map init successful!! **************" << std::endl;
    std::cout << "---------------------------------------------------" << std::endl;

    nodesptr_ = vectormap_->GetNodesConstPtr();
    waysptr_ = vectormap_->GetWaysConstPtr();
    relationsptr_ = vectormap_->GetRelationConstPtr();
    centerwaysptr_ = vectormap_->GetCenterwayConstPtr();
    //gridmapsptr_ = vectormap_->GetGridmapConstPtr();

    //test gridmap
    //grid_map::GridMapRosConverter::toMessage(*gridmapsptr, {"obstacle"}, gridmapmsg);//{"obstacle", "distance"}
    //grid_map::GridMapRosConverter::toOccupancyGrid(*gridmapsptr, "obstacle", 255, 0, gridmapmsg);

    //基准点设置，默认初始时不存在起点、终点
    isstart_path_exist_ = false;
    isend_path_exist_ = false;
    start_centerpoint_id_ = -1;
    end_centerpoint_id_ = -2;
    atnowpoint_ = new node::Point3D(0, 0);
    imucount_ = 0;
    imuinit_flag_ = false;
    endPose_.position.x = 0;
    endPose_.position.y = 0;
    endPose_.orientation.w = 1;

    //globalplan
    //若const对象想调用非const成员函数，则需要进行强制类型转换const_cast <T&>(Obj)
    //若const成员函数想调用非const成员函数，则需要对this指针进行强制类型转换const_cast <T&>(*this)
    globalplans_ = new plan::Globalplan(const_cast<centerway::CenterWay*>(centerwaysptr_));

    //visualization_msgs::Marker
    visualmap_ = new MapVisualization();
    visualmap_->Map2Marker(nodesptr_, waysptr_, centerwaysptr_, relationsptr_, map_markerarray_, ros::Time::now());
    
    startpoint_sub_ = n_.subscribe("/initialpose", 1, &HDMap::StartpointCallback, this);
    goalpoint_sub_ = n_.subscribe("/move_base_simple/goal", 1, &HDMap::GoalpointCallback, this);
    gps_sub_ = n_.subscribe("/comb", 10, &HDMap::GpsCallback, this);
    // gps_sub_ = n_.subscribe("/mapping_odometry", 1, &HDMap::GpsCallback, this);
    //main loop
    ros::Rate r(10);
    while(n_.ok())
    {
        //visualmap->run();
        currenttime_ = ros::Time::now();
        map_pub_.publish(map_markerarray_);
        path_pub_.publish(path_markerarray_);
        // gridmapmsg.header.stamp = currenttime;
        // gridmap_pub.publish(gridmapmsg);
        // navigation_pub.publish(laneletinfo);
        //到达终点退出程序
        if(start_centerpoint_id_ == end_centerpoint_id_)
        {
            std::cout << "***************************************************" << std::endl;
            std::cout << std::endl;
            std::cout << "               arrvied at goal point               " << std::endl;
            std::cout << std::endl;
            std::cout << "***************************************************" << std::endl;
            break;
        }
        ros::spinOnce();
        r.sleep();
    }
}

void HDMap::PushCenterPoint(const std::vector<int> &pathid)
{
    smoothpathnode_.clear();
    if(pathid.empty()) return;

    smoothpathnode_.push_back(centerway::CenterPoint3D(*atnowpoint_));
    centerway::CenterPoint3D pre_centerway_point = smoothpathnode_.back();
    double accumulatelength = 0;
    for(int i = 0; i < pathid.size(); ++i)
    {
        centerway::CenterWay3D *oneway = centerwaysptr_->Find(pathid[i]);

        int j = 0;
        if(i == 0) while(j < oneway->length_ - 1 && oneway->centernodeline_[j] != start_centerpoint_id_) j++;
        for(; j < oneway->length_ - 1; ++j)
        {
            smoothpathnode_.push_back(*centerwaysptr_->FindCenterPoint(oneway->centernodeline_[j]));
            if(smoothpathnode_.size() == 2)
            {
                smoothpathnode_[0].ele_ = smoothpathnode_[1].ele_;
                pre_centerway_point.ele_ = smoothpathnode_[1].ele_;
            }
            //截取100米
            accumulatelength += centerwaysptr_->NodeDistance2D(&pre_centerway_point, &smoothpathnode_.back());
            pre_centerway_point = smoothpathnode_.back();
            if(accumulatelength > 100) break;
            if(i == pathid.size() - 1 && oneway->centernodeline_[j] == end_centerpoint_id_) break;
        }
        if(accumulatelength > 100) break;
    }

    //对终点、起点后处理
    if(accumulatelength < 100)
    {
        smoothpathnode_.erase(smoothpathnode_.end());
        smoothpathnode_.push_back(centerway::CenterPoint3D(end_state_[0], end_state_[1]));
        smoothpathnode_.back().ele_ = smoothpathnode_[1].ele_;
    }
    if(smoothpathnode_.size() >= 3) smoothpathnode_.erase(smoothpathnode_.begin()+1);
}

void HDMap::SmoothPath()
{
    //B样条插值
    //std::cout << "smoothpathnode size: " << smoothpathnode.size() << std::endl;
    // int *intnum = new int[smoothpathnode.size() - 1];
    // for(int i = 0; i < smoothpathnode.size() - 1; ++i)
    // {
    //     intnum[i] = 5;
    // }
    // int num2 = smoothpathnode.size();
    // plan::CBSpline cbspline;
    // cbspline.ThreeOrderBSplineInterpolatePt(smoothpathnode, num2, intnum);
    // delete []intnum;
    //std::cout << "smoothpathnode after size: " << smoothpathnode.size() << std::endl;

    //cubic_spline
    if(smoothpathnode_.size() < 2) return;
    plan::Spline2D csp_obj(smoothpathnode_);
    std::vector<centerway::CenterPoint3D> temppathnode;
    std::vector<double> rcurvature;
    for(double i = 0; i < csp_obj.s.back(); i += 0.2)
    {
        std::array<double, 3> point = csp_obj.calc_postion(i);
        centerway::CenterPoint3D pointtemp;
        pointtemp.x_ = point[0];
        pointtemp.y_ = point[1];
        pointtemp.ele_ = point[2];
        temppathnode.push_back(pointtemp);
        rcurvature.push_back(csp_obj.calc_curvature(i));
    }
    smoothpathnode_ = std::move(temppathnode);

    //osqp
    // if(smoothpathnode_.size() < 2) return;
    // plan::ReferencePathSmoother smooth(smoothpathnode_);
    // smooth.Smooth();
    // std::vector<double> tempx, tempy, tempz;
    // tempx = smooth.GetXList();
    // tempy = smooth.GetYList();
    // tempz = smooth.GetZList();
    // std::vector<centerway::CenterPoint3D> temppathnode;
    // for(int i = 0; i < tempx.size(); ++i)
    // {
    //     centerway::CenterPoint3D pointtemp;
    //     pointtemp.x_ = tempx[i];
    //     pointtemp.y_ = tempy[i];
    //     pointtemp.ele_ = tempz[i];
    //     temppathnode.emplace_back(pointtemp);
    // }
    // smoothpathnode_ = std::move(temppathnode);
}

void HDMap::FullNavigationInfo()
{
    //计算起点与道路边界的距离
    centerway::CenterPoint3D startpoint(atnowpoint_->local_x_, atnowpoint_->local_y_);
    map::relation::relationship *relation_temp = relationsptr_->Find(start_path_);
    double leftdis = globalplans_->Point2EdgeDistance(startpoint, nodesptr_, waysptr_->Find(relation_temp->leftedge_.ID_), start_path_);
    double rightdis = globalplans_->Point2EdgeDistance(startpoint, nodesptr_, waysptr_->Find(relation_temp->rightedge_.ID_), start_path_);
    //std::cout << "start point to left distance: " << leftdis << ", to right distence: " << rightdis << std::endl;

    std::vector<int> outneighbors;
    centerwaysptr_->FindNeighbor(start_path_, outneighbors);
    //填充laneletinfo
    laneletinfo_.CurrentSequenceIDs.clear();
    laneletinfo_.TrafficSign.clear();

    laneletinfo_.header.frame_id = "map";
    laneletinfo_.header.stamp = currenttime_;
    for(int i = 0; i < outneighbors.size(); ++i)
    {
        laneletinfo_.CurrentSequenceIDs.push_back(outneighbors[i]);
    }
    laneletinfo_.TargetSequeceIDs = start_path_;
    laneletinfo_.SpeedLimits = relation_temp->speed_limit_;
    laneletinfo_.ToLeftDistance = leftdis;
    laneletinfo_.ToRightDistance = rightdis;
    laneletinfo_.Direction = static_cast<int>(relation_temp->turn_direction_);
    laneletinfo_.IntersectionDistance = centerwaysptr_->Length2Intersection(start_centerpoint_id_, paths_, relationsptr_);
    
    std::vector<map::relation::regulatoryelement*> trafficsigninfo = relationsptr_->GetRegulatoryElement(start_path_);
    for(int i = 0; i < trafficsigninfo.size(); ++i)
    {
        osmmap::Regulatoryelement onesign;
        onesign.SignType = static_cast<int>(trafficsigninfo[i]->subtype_);
        onesign.LaneletID = trafficsigninfo[i]->laneletid_;
        onesign.CenterpointID = trafficsigninfo[i]->centerpoint3did_;
        laneletinfo_.TrafficSign.push_back(onesign);
    }
}

void HDMap::FullLanesInfo(const int id)
{
    lanesinfo_.data.clear();
    lanesinfo_.header.frame_id = "map";
    lanesinfo_.header.stamp = currenttime_;
    std::vector<int> nextlaneids = globalplans_->FindNextLanes(id);
    for(int i = 0; i < nextlaneids.size(); ++i)
    {
        osmmap::Lane alane;
        alane.laneletid = nextlaneids[i];
        int leftlineid = relationsptr_->Find(id)->leftedge_.ID_;
        int rightlineid = relationsptr_->Find(id)->rightedge_.ID_;
        //左车道
        for(int j = 0; j < waysptr_->Find(leftlineid)->length_; ++j)
        {
            geometry_msgs::Point p;
            p.x = nodesptr_->Find(waysptr_->Find(leftlineid)->nodeline_[j])->local_x_;
            p.y = nodesptr_->Find(waysptr_->Find(leftlineid)->nodeline_[j])->local_y_;
            p.z = nodesptr_->Find(waysptr_->Find(leftlineid)->nodeline_[j])->elevation_;
            alane.leftpoints.push_back(p);
        }
        //右车道
        for(int j = 0; j < waysptr_->Find(rightlineid)->length_; ++j)
        {
            geometry_msgs::Point p;
            p.x = nodesptr_->Find(waysptr_->Find(rightlineid)->nodeline_[j])->local_x_;
            p.y = nodesptr_->Find(waysptr_->Find(rightlineid)->nodeline_[j])->local_y_;
            p.z = nodesptr_->Find(waysptr_->Find(rightlineid)->nodeline_[j])->elevation_;
            alane.rightpoints.push_back(p);
        }
        lanesinfo_.data.push_back(alane);
    }
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> HDMap::GetSkewMatrix(const Eigen::MatrixBase<Derived> &v)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> w;
    w << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;
    return w;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> HDMap::Amatrix(const Eigen::MatrixBase<Derived> &v)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> w = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
    typename Derived::Scalar norm = v.norm();
    Eigen::Matrix<typename Derived::Scalar, 3, 3> skew = GetSkewMatrix(v / norm);

    if (norm < 10e-5)
    {
        return w;
    }

    return Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + (1 - cos(norm)) / norm * skew + (1 - sin(norm) / norm) * skew * skew;
}

void HDMap::ImuInit(const Eigen::Vector3d &imuMsg)
{
    if (++imucount_ < 101)
    {
        Eigen::Vector3d acc{imuMsg[0], imuMsg[1], imuMsg[2]};
        sum_acc_ += acc;
    }
    else if (!imuinit_flag_)
    {
        avg_acc_ = sum_acc_ / 100.0;
        std::cout << "avg acc:  " << avg_acc_.transpose().x() << "  " << avg_acc_.transpose().y()
                  << "  " << avg_acc_.transpose().z() << std::endl;
        double norm = avg_acc_.norm();
        avg_acc_ /= norm;
        Eigen::Vector3d gravity_acc(Eigen::Vector3d(0, 0, 1));
        double cos_value = avg_acc_.dot(gravity_acc);
        double angle = acos(cos_value);
        Eigen::Vector3d axis = GetSkewMatrix(avg_acc_) * gravity_acc;
        // rot = Amatrix(angle * axis);
        Eigen::AngleAxisd rot_vec(angle, axis.normalized());
        rot_ = rot_vec.toRotationMatrix();
        std::cout << "rot: " << rot_ << std::endl;
        imuinit_flag_ = true;
    }
    else
    {
        Eigen::Vector3d acc{imuMsg[0], imuMsg[1], imuMsg[2]};
        adjusted_acc_ = rot_ * acc;
        std::cout << "adjust acc:  " << adjusted_acc_.transpose().x() << "  " << adjusted_acc_.transpose().y()
                  << "  " << adjusted_acc_.transpose().z() << std::endl;
    }
}

void HDMap::OutMapPlan(const centerway::CenterPoint3D &atnow_centerpoint, const double heading)
{
    int laneletid = centerwaysptr_->FindNearestLanelet(&atnow_centerpoint, heading);
    if(laneletid == -1)
    {
        ROS_WARN("can not plan!");
        return;
    }
    start_path_ = laneletid;
    start_centerpoint_id_ = globalplans_->AtWhichPoint(atnow_centerpoint, centerwaysptr_->Find(laneletid));
    if(isend_path_exist_)
    {
        //visualmap_->PathMarkerclear();
        paths_.clear();
        paths_ = globalplans_->Run(start_path_, end_path_);
        if(!paths_.empty())
        {
            visualmap_->Path2Marker(centerwaysptr_, paths_, path_markerarray_, currenttime_);
            PushCenterPoint(paths_);
            //smooth
            // SmoothPath();
            visualmap_->Smoothpath2Marker(smoothpathnode_, path_markerarray_, currenttime_);
            //发布导航信息
            FullNavigationInfo();
            navigation_pub_.publish(laneletinfo_);
            //发路径
            visualization_msgs::Marker golbalpath = path_markerarray_.markers.back();
            golbalpath_pub_.publish(golbalpath);
            //std::cout << "path_markerarray size is " << path_markerarray_.markers.size() << std::endl;
            //发下一个车道
            FullLanesInfo(start_path_);
            lanes_pub_.publish(lanesinfo_);
        }
    }else{
        ROS_WARN("goal point has not set!");
    } 
}

void HDMap::StartpointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double xx = msg->pose.pose.position.x;
    double yy = msg->pose.pose.position.y;
    centerway::CenterPoint3D startpoint(xx, yy);
    int startpoint_res = globalplans_->InWhichCenterway(startpoint, nodesptr_, waysptr_, relationsptr_);
    start_state_[0] = xx;
    start_state_[1] = yy;
    start_state_[2] = tf::getYaw(msg->pose.pose.orientation);// [-pi,pi]
    //std::cout << "start yaw is " << start_state[2] << std::endl;
    ROS_INFO("start point is in %d path", startpoint_res);

    //暂时把起点的位置当作GPS定位信息test
    atnowpoint_->local_x_ = xx;
    atnowpoint_->local_y_ = yy;

    //plan
    if(startpoint_res != -1)
    {
        isstart_path_exist_ = true;
        start_path_ = startpoint_res;
        start_centerpoint_id_ = globalplans_->AtWhichPoint(startpoint, centerwaysptr_->Find(startpoint_res));
        ROS_INFO("start point is at %d", start_centerpoint_id_);
        if(isend_path_exist_)
        {
            //visualmap_->pathmarkerclear();
            path_markerarray_.markers.clear();
            paths_.clear();
            paths_ = globalplans_->Run(start_path_, end_path_);
            if(!paths_.empty())
            {
                visualmap_->Path2Marker(centerwaysptr_, paths_, path_markerarray_, currenttime_);
                PushCenterPoint(paths_);
                //可视化
                visualmap_->Smoothpath2Marker(smoothpathnode_, path_markerarray_, currenttime_);
                //发布导航信息
                FullNavigationInfo();
                navigation_pub_.publish(laneletinfo_);
            }

            //该点到下一个路口(下一次转向)距离测试
            //double intersectiondis = centerways->length2intersection(start_centerpoint_id_, paths);
            //std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;

        }else{
            ROS_WARN("goal point has not set!");
        } 
    }else{
        ROS_WARN("start point out of HD-map!");
    }
}

void HDMap::GoalpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double xx = msg->pose.position.x;
    double yy = msg->pose.position.y;
    centerway::CenterPoint3D goalpoint(xx, yy);
    int goalpoint_res = globalplans_->InWhichCenterway(goalpoint, nodesptr_, waysptr_, relationsptr_);
    end_state_[0] = xx;
    end_state_[1] = yy;
    end_state_[2] = tf::getYaw(msg->pose.orientation);// [-pi,pi]
    endPose_ = msg->pose;
    ROS_INFO("goal point is in %d path", goalpoint_res);

    //plan
    if(goalpoint_res != -1)
    {
        //该点最近邻中心线点测试
        //int closestpointid = globalplans_->AtWhichPoint(goalpoint, centerways_->Find(goalpoint_res));
        //std::cout << "this point closest point id is: " << closestpointid << std::endl;
        
        //该点到边界距离测试
        // map::relation::relationship *relation_temp_ = relations_->Find(goalpoint_res);
        // double leftdis = globalplans_->Point2EdgeDistance(goalpoint_, nodes_, ways_->Find(relation_temp_->leftedge_.ID_), goalpoint_res);
        // double rightdis = globalplans_->Point2EdgeDistance(goalpoint_, nodes_, ways_->Find(relation_temp_->rightedge_.ID_), goalpoint_res);
        // std::cout << "to left distance: " << leftdis << ", to right distence: " << rightdis << std::endl;
        
        isend_path_exist_ = true;
        end_path_ = goalpoint_res;
        end_centerpoint_id_ = globalplans_->AtWhichPoint(goalpoint, centerwaysptr_->Find(goalpoint_res));
        ROS_INFO("goal point is at %d", end_centerpoint_id_);
        if(isstart_path_exist_)
        {
            //visualmap_->pathmarkerclear();
            path_markerarray_.markers.clear();
            paths_.clear();
            paths_ = globalplans_->Run(start_path_, end_path_);
            if(!paths_.empty())
            {
                visualmap_->Path2Marker(centerwaysptr_, paths_, path_markerarray_, currenttime_);
                PushCenterPoint(paths_);
                visualmap_->Smoothpath2Marker(smoothpathnode_, path_markerarray_, currenttime_);

                //发布导航信息
                FullNavigationInfo();
                navigation_pub_.publish(laneletinfo_);

                //发路径
                visualization_msgs::Marker golbalpath = path_markerarray_.markers.back();
                golbalpath_pub_.publish(golbalpath);
                
                //发下一个车道
                //FullLanesInfo(start_path_);
                //lanes_pub_.publish(lanesinfo_);
            }

            //该点到下一个路口(下一次转向)距离测试, 无视终点，直到在地图找到第一个转向路段s
            //double intersectiondis = centerways_->Length2Intersection(start_centerpoint_id, paths_);
            //std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;
            
        }else{
            ROS_WARN("start point has not set!");
        }
    }else{
        ROS_WARN("goal point out of HD-map!");
    }
    
}

void HDMap::GpsCallback(const fsd_common_msgs::Comb::ConstPtr &msg)
// void HDMap::GpsCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(use_gnss_)
    {
        // GNSS
        atnowpoint_->elevation_ = msg->Altitude;
        atnowpoint_->latitude_ = msg->Lattitude;
        atnowpoint_->longitude_ = msg->Longitude;
        vectormap_->GPS2Localxy(atnowpoint_);
    }else{
        //点云配准
        // atnowpoint_->local_x_ = msg->pose.pose.position.x;
        // atnowpoint_->local_y_ = msg->pose.pose.position.y;
        // atnowpoint_->elevation_ = msg->pose.pose.position.z;
    }

    start_state_[0] = atnowpoint_->local_x_;
    start_state_[1] = atnowpoint_->local_y_;
    if(use_gnss_)
    {
        start_state_[2] = (msg->Heading + 90)/180*M_PI;
        
        //imu矫正
        Eigen::Vector3d imuinmsg;
        imuinmsg << msg->Ax, -msg->Az, msg->Ay;
        ImuInit(imuinmsg);
    }else{
        // start_state_[2] = tf::getYaw(msg->pose.pose.orientation);

        //点云配准不矫正imu
    }
    

    //当前点定位到路段
    centerway::CenterPoint3D atnowcenterpoint = centerway::CenterPoint3D(*atnowpoint_);
    //当前路段精确定位到某点
    //int atnowcenterway = globalplans_->InWhichCenterway(atnowcenterpoint, nodesptr, waysptr, relationsptr);
    std::vector<int> lanelets_res = globalplans_->LocateLanelets(atnowcenterpoint, nodesptr_, waysptr_, relationsptr_);
    //ROS_INFO("lanelets number is %d!", (int)lanelets_res.size());

    //HDmap state
    osmmap::CarState hdmapstate;
    hdmapstate.header.frame_id = "map";
    hdmapstate.header.stamp = ros::Time::now();
    hdmapstate.inMap = false;
    hdmapstate.isEndExist = false;
    hdmapstate.isFindPath = false;
    hdmapstate.laneletid = 0;
    //plan
    //visualmap->pathmarkerclear();
    path_markerarray_.markers.clear();
    if(!lanelets_res.empty())
    {
        hdmapstate.inMap = true;

        //融合上一帧路径
        if(paths_.empty())
        {
            //第一帧，优先选取第一个定位结果
            start_path_ = lanelets_res[0];
            // for(int i = 0; i < lanelets_res.size(); i++)
            // {
            //     start_path_ = lanelets_res[i];
            //     if(!globalplans_->Run(start_path_, end_path_).empty())
            //     {
            //         break;
            //     }
            // }
            std::cout << "first plan!" << std::endl;
        }else{
            //其他帧，优先选取上一帧路径lanelet范围内的结果
            bool isInLastPaths = false;
            for(int i = 0; i < lanelets_res.size(); i++)
            {
                if(std::find(paths_.begin(), paths_.end(), lanelets_res[i]) != paths_.end())
                {
                    start_path_ = lanelets_res[i];
                    isInLastPaths = true;
                    std::cout << "select last frame lanelet id successed!" << std::endl;
                    break;
                }
            }
            //如果均不在上一帧lanelet范围内, 优先选取第一个
            if(!isInLastPaths)
            {
                // for(int i = 0; i < lanelets_res.size(); i++)
                // {
                //     start_path_ = lanelets_res[i];
                //     if(!globalplans_->Run(start_path_, end_path_).empty())
                //     {
                //         break;
                //     }
                // }
                start_path_ = lanelets_res[0];
            }
        }
        // start_path_ = atnowcenterway;
        
        hdmapstate.laneletid = start_path_;
        start_centerpoint_id_ = globalplans_->AtWhichPoint(atnowcenterpoint, centerwaysptr_->Find(start_path_));
        if(isend_path_exist_)
        {
            hdmapstate.isEndExist = true;
            //visualmap->pathmarkerclear();
            paths_.clear();
            paths_ = globalplans_->Run(start_path_, end_path_);
            if(!paths_.empty())
            {
                hdmapstate.isFindPath = true;
                visualmap_->Path2Marker(centerwaysptr_, paths_, path_markerarray_, currenttime_);
                PushCenterPoint(paths_);
                //smooth
                // SmoothPath();
                visualmap_->Smoothpath2Marker(smoothpathnode_, path_markerarray_, currenttime_);
                //发布导航信息
                FullNavigationInfo();
                navigation_pub_.publish(laneletinfo_);
                //发路径
                visualization_msgs::Marker golbalpath = path_markerarray_.markers.back();
                golbalpath_pub_.publish(golbalpath);
                //std::cout << "path_markerarray size is " << path_markerarray.markers.size() << std::endl;
                //发下一个车道
                FullLanesInfo(start_path_);
                lanes_pub_.publish(lanesinfo_);
            }else{
                smoothpathnode_.clear();
                smoothpathnode_.push_back(centerway::CenterPoint3D(*atnowpoint_));
                // double atnowpoint_heading = ConstrainAngle(msg->Heading)/180*M_PI;
                double atnowpoint_heading = start_state_[2];
                OutMapPlan(atnowcenterpoint, atnowpoint_heading);
                ROS_WARN("gps point is in reverse direction path!");
            }

            //该点到下一个路口(下一次转向)距离测试
            //double intersectiondis = centerways->length2intersection(start_centerpoint_id_, paths, relations);
            //std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;

        }else{
            ROS_WARN("goal point has not set!");
        } 
    }else{
        //越界：直接连接当前点与最近的道路中心点！！
        smoothpathnode_.clear();
        smoothpathnode_.push_back(centerway::CenterPoint3D(*atnowpoint_));
        // double atnowpoint_heading = ConstrainAngle(msg->Heading)/180*M_PI;
        double atnowpoint_heading = start_state_[2];
        
        // int nearestcenterwayid = centerwaysptr->findNearestCenterwaypointid(atnowpoint);
        // centerway::CenterPoint3D tartgetpoint;
        // centerwaysptr->nextCenterwayPointid(nearestcenterwayid, tartgetpoint);
        // smoothpathnode.push_back(tartgetpoint);

        // visualmap->smoothpath2marker(smoothpathnode, path_markerarray, currenttime);
        // //发路径
        // visualization_msgs::Marker golbalpath_ = path_markerarray.markers.back();
        // golbalpath_pub.publish(golbalpath_);
        OutMapPlan(atnowcenterpoint, atnowpoint_heading);
        
        ROS_WARN("gps point out of HD-map!");
    }

    //tf
    // double headtemp = msg->Heading;
    // headtemp = ConstrainAngle(headtemp);
    double headtemp = start_state_[2];
    // std::cout << "headtemp: " << headtemp << std::endl;
    tf::Quaternion qenu2base;

    //GNSS
    qenu2base.setRPY((msg->Roll)/180*M_PI, (msg->Pitch)/180*M_PI, (msg->Heading+90)/180*M_PI);

    //点云配准
    // qenu2base.setW(msg->pose.pose.orientation.w);
    // qenu2base.setX(msg->pose.pose.orientation.x);
    // qenu2base.setY(msg->pose.pose.orientation.y);
    // qenu2base.setZ(msg->pose.pose.orientation.z);
    baselink2map_.setRotation(qenu2base);
    baselink2map_.setOrigin(tf::Vector3(atnowpoint_->local_x_, atnowpoint_->local_y_, atnowpoint_->elevation_));
    //map->rslidar->base_link
    broadcaster_.sendTransform(tf::StampedTransform(baselink2map_, ros::Time::now(), "map", "rslidar"));
    std::cout << "atnowpoint update" << std::endl;
    std::cout << "now x: " << atnowpoint_->local_x_ << ", now y: " << atnowpoint_->local_y_ << ", now z: " << atnowpoint_->elevation_ << std::endl;

    //Ve, Vn, Vu 转化为车辆坐标系速度
    Eigen::Matrix3d m3d;
    m3d << 1, 0, 0, 0, 0, -1, 0, -1, 0;
    Eigen::Quaterniond qbase2imu(m3d);
    Eigen::Quaterniond q1 = Eigen::Quaterniond(qenu2base.w(), qenu2base.x(), qenu2base.y(), qenu2base.z()).normalized();
    Eigen::Vector3d Venu = Eigen::Vector3d(msg->Ve, msg->Vn, msg->Vu);
    Eigen::Vector3d Vxyz = (q1*qbase2imu).normalized().inverse() * Venu;

    //debug
    // std::cout << "vx=" << Vxyz(0) << ", vy=" << Vxyz(1) << ", vz=" << Vxyz(2) << std::endl;
    // std::cout << "ax=" << msg->Ax << ", ay=" << msg->Ay << ", az=" << msg->Az << std::endl;
    // double asum = std::sqrt(msg->Ax * msg->Ax + msg->Ay * msg->Ay + msg->Az * msg->Az);
    // std::cout << "asum=" << asum << std::endl;

    //欧拉角转四元数
    // double siny_cosp = +2.0 * (q1.w() * q1.z() + q1.x() * q1.y());
    // double cosy_cosp = +1.0 - 2.0 * (q1.y() * q1.y() + q1.z() * q1.z());
    // double yaw = atan2(siny_cosp, cosy_cosp);
    // std::cout << "head: " << (headtemp)/180*3.14159 << ", yaw: " << yaw << std::endl;
    
    //smoothpath
    // if(smoothpathnode.size() >= 2)
    // {
        // plan::Spline2D csp_obj(smoothpathnode);
        // std::vector<centerway::CenterPoint3D> temppathnode;
        // std::vector<double> rcurvature;
        // for(double i = 0; i < csp_obj.s.back(); i += 0.2)
        // {
        //     std::array<double, 3> point_ = csp_obj.calc_postion(i);
        //     centerway::CenterPoint3D pointtemp;
        //     pointtemp.x = point_[0];
        //     pointtemp.y = point_[1];
        //     pointtemp.ele = point_[2];
        //     temppathnode.push_back(pointtemp);
        //     rcurvature.push_back(csp_obj.calc_curvature(i));
        // }
    // }

    //gps path
    gpspath_.header.frame_id = "map";
    gpspath_.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = atnowpoint_->local_x_;
    pose_stamped.pose.position.y = atnowpoint_->local_y_;
    pose_stamped.pose.position.z = atnowpoint_->elevation_;
    pose_stamped.pose.orientation.x = qenu2base.x();
    pose_stamped.pose.orientation.y = qenu2base.y();
    pose_stamped.pose.orientation.z = qenu2base.z();
    pose_stamped.pose.orientation.w = qenu2base.w();
    gpspath_.poses.push_back(pose_stamped);
    gpspath_pub_.publish(gpspath_);

    //car state
    hdmapstate.heading = headtemp;
    hdmapstate.carPose.position.x = atnowpoint_->local_x_;
    hdmapstate.carPose.position.y = atnowpoint_->local_y_;
    hdmapstate.carPose.position.z = atnowpoint_->elevation_;
    hdmapstate.carPose.orientation.x = qenu2base.x();
    hdmapstate.carPose.orientation.y = qenu2base.y();
    hdmapstate.carPose.orientation.z = qenu2base.z();
    hdmapstate.carPose.orientation.w = qenu2base.w();
    hdmapstate.endPose = endPose_;
    hdmapstate.nextpoint.x = 0;
    hdmapstate.nextpoint.y = 0;
    hdmapstate.nextpoint.z = 0;

    //给hybid A*终点
    if(smoothpathnode_.size() >= 2) 
    {
        plan::Spline2D csp_obj(smoothpathnode_);
        if(csp_obj.s.back() >= 6)
        {
            std::array<double, 3> point_ = csp_obj.calc_postion(6);
            hdmapstate.nextpoint.x = point_[0];
            hdmapstate.nextpoint.y = point_[1];
            hdmapstate.nextpoint.z = csp_obj.calc_yaw(6);
        }
    }
    // if(!path_markerarray.markers.empty() && path_markerarray.markers.back().points.size() >= 2)
    // {
    //     hdmapstate.nextpoint.x = path_markerarray.markers.back().points[1].x;
    //     hdmapstate.nextpoint.y = path_markerarray.markers.back().points[1].y;
    // }
    // if(!path_markerarray.markers.empty() && path_markerarray.markers.back().points.size() == 2)
    // {
    //     double dx = path_markerarray.markers.back().points[1].x - atnowpoint->local_x;
    //     double dy = path_markerarray.markers.back().points[1].y - atnowpoint->local_y;
    //     if(dx < 1e-5)
    //     {
    //         hdmapstate.nextpoint.z = dy > 0 ? M_PI : -M_PI;
    //     }else{
    //         hdmapstate.nextpoint.z = atan2(dy, dx);
    //     }
    // }else if(!path_markerarray.markers.empty() && path_markerarray.markers.back().points.size() > 2){
    //     double dx = path_markerarray.markers.back().points[2].x - path_markerarray.markers.back().points[1].x;
    //     double dy = path_markerarray.markers.back().points[2].y - path_markerarray.markers.back().points[1].y;
    //     if(dx < 1e-5)
    //     {
    //         hdmapstate.nextpoint.z = dy > 0 ? M_PI : -M_PI;
    //     }else{
    //         hdmapstate.nextpoint.z = atan2(dy, dx);
    //     }
    // }

    if(use_gnss_)
    {
        //GNSS
        hdmapstate.linear.x = Vxyz(0);
        hdmapstate.linear.y = Vxyz(1);
        hdmapstate.linear.z = Vxyz(2);
        hdmapstate.Accell.x = msg->Ax;
        hdmapstate.Accell.y = msg->Ay;
        hdmapstate.Accell.z = msg->Az;
    }else{
        //点云配准
        // hdmapstate.linear.x = msg->twist.twist.linear.x;
        // hdmapstate.linear.y = msg->twist.twist.linear.y;
        // hdmapstate.linear.z = msg->twist.twist.linear.z;
        // hdmapstate.angular.x = msg->twist.twist.angular.x;
        // hdmapstate.angular.y = msg->twist.twist.angular.y;
        // hdmapstate.angular.z = msg->twist.twist.angular.z;
    }

    hdmapstate.Accell.x = 0;
    hdmapstate.Accell.y = 0;
    hdmapstate.Accell.z = 0;
    if(imuinit_flag_)
    {
        hdmapstate.Accell.x = adjusted_acc_[0];
        hdmapstate.Accell.y = adjusted_acc_[1];
        hdmapstate.Accell.z = adjusted_acc_[2];
    }
    carstate_pub_.publish(hdmapstate);

    //
    std_msgs::Float32 sp;
    sp.data = hdmapstate.linear.x;
    speed_pub_.publish(std::move(sp));
}

HDMap::~HDMap()
{
    delete atnowpoint_;
    delete visualmap_;
    delete globalplans_;
    delete vectormap_;
}


} // namespace map
