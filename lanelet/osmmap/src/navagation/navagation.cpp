/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 19:32:51
 * @LastEditTime: 2022-11-23 18:53:04
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/navagation/navagation.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "navagation/navagation.h"

namespace navagation
{
NavagationBase::NavagationBase(ros::NodeHandle &n):n_(n)
{
    std::string file_path_, file_name_;
    double origin_lat_, origin_lon_, origin_ele_;
    n_.getParam("file_path", file_path_);
    n_.getParam("file_name", file_name_);
    n_.getParam("origin_lat", origin_lat_);
    n_.getParam("origin_lon", origin_lon_);
    n_.getParam("origin_ele", origin_ele_);

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
    vectormap_ = new map::Map(file_path_, file_name_);
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
    atnowpoint_ = new map::node::Point3D(0, 0);
    endPose_.position.x = 0;
    endPose_.position.y = 0;
    endPose_.orientation.w = 1;

    //globalplan
    //若const对象想调用非const成员函数，则需要进行强制类型转换const_cast <T&>(Obj)
    //若const成员函数想调用非const成员函数，则需要对this指针进行强制类型转换const_cast <T&>(*this)
    globalplans_ = new plan::Globalplan(const_cast<map::centerway::CenterWay*>(centerwaysptr_));

    //visualization_msgs::Marker
    visualmap_ = new map::MapVisualization();
    visualmap_->Map2Marker(nodesptr_, waysptr_, centerwaysptr_, relationsptr_, map_markerarray_, ros::Time::now());
    
    startpoint_sub_ = n_.subscribe("/initialpose", 1, &NavagationBase::StartpointCallback, this);
    goalpoint_sub_ = n_.subscribe("/move_base_simple/goal", 1, &NavagationBase::GoalpointCallback, this);
}

NavagationBase::~NavagationBase()
{
    delete atnowpoint_;
    delete visualmap_;
    delete globalplans_;
    delete vectormap_;
}

void NavagationBase::FullNavigationInfo()
{
    //计算起点与道路边界的距离
    map::centerway::CenterPoint3D startpoint(atnowpoint_->local_x_, atnowpoint_->local_y_);
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

void NavagationBase::FullLanesInfo(const int id)
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

void NavagationBase::PushCenterPoint(const std::vector<int> &pathid)
{
    smoothpathnode_.clear();
    if(pathid.empty()) return;

    // smoothpathnode_.push_back(map::centerway::CenterPoint3D(*atnowpoint_));
    // map::centerway::CenterPoint3D pre_centerway_point = smoothpathnode_.back();
    map::centerway::CenterPoint3D pre_centerway_point;
    double accumulatelength = 0;
    for(int i = 0; i < pathid.size(); ++i)
    {
        map::centerway::CenterWay3D *oneway = centerwaysptr_->Find(pathid[i]);

        int j = 0;
        if(i == 0) while(j < oneway->length_ - 1 && oneway->centernodeline_[j] != start_centerpoint_id_) j++;
        for(; j < oneway->length_ - 1; ++j)
        {
            smoothpathnode_.push_back(*centerwaysptr_->FindCenterPoint(oneway->centernodeline_[j]));
            // if(smoothpathnode_.size() == 2)
            // {
                // smoothpathnode_[0].ele_ = smoothpathnode_[1].ele_;
                // pre_centerway_point.ele_ = smoothpathnode_[1].ele_;
            // }
            //截取100米，包括车辆自身
            // accumulatelength += centerwaysptr_->NodeDistance2D(&pre_centerway_point, &smoothpathnode_.back());
            // pre_centerway_point = smoothpathnode_.back();
            //纯参考线
            if(smoothpathnode_.size() > 1)
            {
                accumulatelength += centerwaysptr_->NodeDistance2D(&pre_centerway_point, &smoothpathnode_.back());
            }
            pre_centerway_point = smoothpathnode_.back();

            if(accumulatelength > 100) break;
            if(i == pathid.size() - 1 && oneway->centernodeline_[j] == end_centerpoint_id_) break;
        }
        if(accumulatelength > 100) break;
    }

    //对终点、起点后处理
    // if(accumulatelength < 100)
    // {
    //     smoothpathnode_.erase(smoothpathnode_.end());
    //     smoothpathnode_.push_back(map::centerway::CenterPoint3D(end_state_[0], end_state_[1]));
    //     smoothpathnode_.back().ele_ = smoothpathnode_[1].ele_;
    // }
    // if(smoothpathnode_.size() >= 3) smoothpathnode_.erase(smoothpathnode_.begin()+1);
}

void NavagationBase::OutMapPlan(const map::centerway::CenterPoint3D &atnow_centerpoint, const double heading)
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
            SmoothPath();
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

void NavagationBase::SmoothPath()
{
    //B样条插值
    // std::cout << "smoothpathnode size: " << smoothpathnode_.size() << std::endl;
    // int *intnum = new int[smoothpathnode_.size() - 1];
    // for(int i = 0; i < smoothpathnode_.size() - 1; ++i)
    // {
    //     intnum[i] = 5;
    // }
    // int num2 = smoothpathnode_.size();
    // plan::CBSpline cbspline;
    // cbspline.ThreeOrderBSplineInterpolatePt(smoothpathnode_, num2, intnum);
    // delete []intnum;
    // std::cout << "smoothpathnode after size: " << smoothpathnode_.size() << std::endl;

    //cubic_spline
    // if(smoothpathnode_.size() < 2) return;
    // plan::Spline2D csp_obj(smoothpathnode_);
    // std::vector<map::centerway::CenterPoint3D> temppathnode;
    // std::vector<double> rcurvature;
    // for(double i = 0; i < csp_obj.s.back(); i += 0.2)
    // {
    //     std::array<double, 3> point = csp_obj.calc_postion(i);
    //     map::centerway::CenterPoint3D pointtemp;
    //     pointtemp.x_ = point[0];
    //     pointtemp.y_ = point[1];
    //     pointtemp.ele_ = point[2];
    //     temppathnode.push_back(pointtemp);
    //     rcurvature.push_back(csp_obj.calc_curvature(i));
    // }
    // smoothpathnode_ = std::move(temppathnode);

    //osqp
    // if(smoothpathnode_.size() < 2) return;
    // plan::ReferencePathSmoother smooth(smoothpathnode_);
    // smooth.Smooth();
    // std::vector<double> tempx, tempy, tempz;
    // tempx = smooth.GetXList();
    // tempy = smooth.GetYList();
    // tempz = smooth.GetZList();
    // std::vector<map::centerway::CenterPoint3D> temppathnode;
    // for(int i = 0; i < tempx.size(); ++i)
    // {
    //     map::centerway::CenterPoint3D pointtemp;
    //     pointtemp.x_ = tempx[i];
    //     pointtemp.y_ = tempy[i];
    //     pointtemp.ele_ = tempz[i];
    //     temppathnode.emplace_back(pointtemp);
    // }
    // smoothpathnode_ = std::move(temppathnode);

    //apollo
    // if(smoothpathnode_.size() < 2) return;
    // apollo::planning::PathSmoother smooth(smoothpathnode_);
    // /// @brief 在process对传入的initnode修正，initnode前两位本身应为0
    // std::array<double, 3> initnode = {0, 0, 0};//s-l坐标系, (l, dl/ds, d(dl/ds)/ds)
    // std::vector<map::centerway::CenterPoint3D> temppathnode;
    // if(smooth.Process(initnode, temppathnode))
    // {
    //     smoothpathnode_ = std::move(temppathnode);
    //     ROS_INFO("apollo smooth successed!");
    // }else{
    //     ROS_WARN("apollo smooth failed!");
    // }
}
    
void NavagationBase::StartpointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    double xx = msg->pose.pose.position.x;
    double yy = msg->pose.pose.position.y;
    map::centerway::CenterPoint3D startpoint(xx, yy);
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

void NavagationBase::GoalpointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double xx = msg->pose.position.x;
    double yy = msg->pose.position.y;
    map::centerway::CenterPoint3D goalpoint(xx, yy);
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




}//namespace navagation