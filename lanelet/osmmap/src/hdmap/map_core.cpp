/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-09-24 15:16:06
 * @LastEditTime: 2022-09-25 16:19:55
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/hdmap/map_core.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "osmmap/map_core.h"

namespace map
{
HDMap::HDMap(ros::NodeHandle n_):n(n_)
{
    std::string file_path_, file_name_;
    double origin_lat_, origin_lon_, origin_ele_;
    n.getParam("file_path", file_path_);
    n.getParam("file_name", file_name_);
    n.getParam("origin_lat", origin_lat_);
    n.getParam("origin_lon", origin_lon_);
    n.getParam("origin_ele", origin_ele_);

    //可视化
    map_pub = n.advertise<visualization_msgs::MarkerArray>("map", 1);
    path_pub = n.advertise<visualization_msgs::MarkerArray>("path", 1);
    gpspath_pub = n.advertise<nav_msgs::Path>("gpspath_info", 1);
    // gridmap_pub = n.advertise<nav_msgs::OccupancyGrid>("gridmap_info", 1, true);
    //导航信息
    navigation_pub = n.advertise<osmmap::Navigation>("navigation_info", 1);
    carstate_pub = n.advertise<osmmap::CarState>("carstate_info", 1);
    lanes_pub = n.advertise<osmmap::Lanes>("lanes_info", 1);
    //全局路径(平滑后)，只在gps_callback调用
    golbalpath_pub = n.advertise<visualization_msgs::Marker>("golbalpath_info", 1);

    std::cout << "---------------------------------------------------" << std::endl;
    vectormap = new Map(file_path_, file_name_);
    vectormap->setOrigin(origin_lat_, origin_lon_, origin_ele_);
    std::cout << "************** map init successful!! **************" << std::endl;
    std::cout << "---------------------------------------------------" << std::endl;

    nodesptr = vectormap->getNodesConstPtr();
    waysptr = vectormap->getWaysConstPtr();
    relationsptr = vectormap->getRelationConstPtr();
    centerwaysptr = vectormap->getCenterwayConstPtr();
    //gridmapsptr = vectormap->getGridmapConstPtr();

    //test gridmap
    //grid_map::GridMapRosConverter::toMessage(*gridmapsptr, {"obstacle"}, gridmapmsg);//{"obstacle", "distance"}
    //grid_map::GridMapRosConverter::toOccupancyGrid(*gridmapsptr, "obstacle", 255, 0, gridmapmsg);

    //基准点设置，默认初始时不存在起点、终点
    isstart_path_exist = false;
    isend_path_exist = false;
    start_centerpoint_id = -1;
    end_centerpoint_id = -2;
    atnowpoint = new node::Point3D(0, 0);
    imucount = 0;
    imuinit_flag = false;
    endPose.position.x = 0;
    endPose.position.y = 0;
    endPose.orientation.w = 1;

    //globalplan
    //若const对象想调用非const成员函数，则需要进行强制类型转换const_cast <T&>(Obj)
    //若const成员函数想调用非const成员函数，则需要对this指针进行强制类型转换const_cast <T&>(*this)
    globalplans = new plan::Globalplan(const_cast<centerway::CenterWay*>(centerwaysptr));

    //visualization_msgs::Marker
    visualmap = new MapVisualization();
    visualmap->map2marker(nodesptr, waysptr, centerwaysptr, relationsptr, map_markerarray, ros::Time::now());
    
    startpoint_sub = n.subscribe("/initialpose", 10, &HDMap::startpoint_callback, this);
    goalpoint_sub = n.subscribe("/move_base_simple/goal", 10, &HDMap::goalpoint_callback, this);
    gps_sub = n.subscribe("/comb", 10, &HDMap::gps_callback, this);
    //main loop
    ros::Rate r(10);
    while(n.ok())
    {
        //visualmap->run();
        currenttime = ros::Time::now();
        map_pub.publish(map_markerarray);
        path_pub.publish(path_markerarray);
        // gridmapmsg.header.stamp = currenttime;
        // gridmap_pub.publish(gridmapmsg);
        // navigation_pub.publish(laneletinfo);
        //到达终点退出程序
        if(start_centerpoint_id == end_centerpoint_id)
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

void HDMap::pushCenterPoint(const std::vector<int> &pathid_)
{
    smoothpathnode.clear();
    if(pathid_.empty()) return;

    smoothpathnode.push_back(centerway::CenterPoint3D(*atnowpoint));
    centerway::CenterPoint3D pre_centerway_point = smoothpathnode.back();
    double accumulatelength_ = 0;
    for(int i = 0; i < pathid_.size(); ++i)
    {
        centerway::CenterWay3D *oneway = centerwaysptr->Find(pathid_[i]);

        int j = 0;
        if(i == 0) while(j < oneway->length - 1 && oneway->centernodeline[j] != start_centerpoint_id) j++;
        for(; j < oneway->length - 1; ++j)
        {
            smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
            if(smoothpathnode.size() == 2)
            {
                smoothpathnode[0].ele = smoothpathnode[1].ele;
                pre_centerway_point.ele = smoothpathnode[1].ele;
            }
            //截取100米
            accumulatelength_ += centerwaysptr->NodeDistance2D(&pre_centerway_point, &smoothpathnode.back());
            pre_centerway_point = smoothpathnode.back();
            if(accumulatelength_ > 100) break;
            if(i == pathid_.size() - 1 && oneway->centernodeline[j] == end_centerpoint_id) break;
        }
        if(accumulatelength_ > 100) break;
    }

    //对终点、起点后处理
    if(accumulatelength_ < 100)
    {
        smoothpathnode.erase(smoothpathnode.end());
        smoothpathnode.push_back(centerway::CenterPoint3D(end_state[0], end_state[1]));
        smoothpathnode.back().ele = smoothpathnode[1].ele;
    }
    if(smoothpathnode.size() >= 3) smoothpathnode.erase(smoothpathnode.begin()+1);
}

void HDMap::Smoothpath()
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
    if(smoothpathnode.size() < 2) return;
    plan::Spline2D csp_obj(smoothpathnode);
    std::vector<centerway::CenterPoint3D> temppathnode;
    std::vector<double> rcurvature;
    for(double i = 0; i < csp_obj.s.back(); i += 0.2)
    {
        std::array<double, 3> point_ = csp_obj.calc_postion(i);
        centerway::CenterPoint3D pointtemp;
        pointtemp.x = point_[0];
        pointtemp.y = point_[1];
        pointtemp.ele = point_[2];
        temppathnode.push_back(pointtemp);
        rcurvature.push_back(csp_obj.calc_curvature(i));
    }
    smoothpathnode = temppathnode;
}

void HDMap::fullNavigationInfo()
{
    //计算起点与道路边界的距离
    centerway::CenterPoint3D startpoint_(atnowpoint->local_x, atnowpoint->local_y);
    map::relation::relationship *relation_temp_ = relationsptr->Find(start_path);
    double leftdis = globalplans->Point2edgedistance(startpoint_, nodesptr, waysptr->Find(relation_temp_->leftedge.ID), start_path);
    double rightdis = globalplans->Point2edgedistance(startpoint_, nodesptr, waysptr->Find(relation_temp_->rightedge.ID), start_path);
    //std::cout << "start point to left distance: " << leftdis << ", to right distence: " << rightdis << std::endl;

    std::vector<int> outneighbors;
    centerwaysptr->findNeighbor(start_path, outneighbors);
    //填充laneletinfo
    laneletinfo.CurrentSequenceIDs.clear();
    laneletinfo.TrafficSign.clear();

    laneletinfo.header.frame_id = "map";
    laneletinfo.header.stamp = currenttime;
    for(int i = 0; i < outneighbors.size(); ++i)
    {
        laneletinfo.CurrentSequenceIDs.push_back(outneighbors[i]);
    }
    laneletinfo.TargetSequeceIDs = start_path;
    laneletinfo.SpeedLimits = relation_temp_->speed_limit;
    laneletinfo.ToLeftDistance = leftdis;
    laneletinfo.ToRightDistance = rightdis;
    laneletinfo.Direction = static_cast<int>(relation_temp_->turn_direction);
    laneletinfo.IntersectionDistance = centerwaysptr->length2intersection(start_centerpoint_id, paths, relationsptr);
    
    std::vector<map::relation::regulatoryelement*> trafficsigninfo = relationsptr->getRegulatoryelement(start_path);
    for(int i = 0; i < trafficsigninfo.size(); ++i)
    {
        osmmap::Regulatoryelement onesign;
        onesign.SignType = static_cast<int>(trafficsigninfo[i]->subtype);
        onesign.LaneletID = trafficsigninfo[i]->laneletid;
        onesign.CenterpointID = trafficsigninfo[i]->centerpoint3did;
        laneletinfo.TrafficSign.push_back(onesign);
    }
}

void HDMap::fullLanesInfo(const int id_)
{
    Lanesinfo.data.clear();
    Lanesinfo.header.frame_id = "map";
    Lanesinfo.header.stamp = currenttime;
    std::vector<int> nextlaneids = globalplans->findNextLanes(id_);
    for(int i = 0; i < nextlaneids.size(); ++i)
    {
        osmmap::Lane alane;
        alane.laneletid = nextlaneids[i];
        int leftlineid = relationsptr->Find(id_)->leftedge.ID;
        int rightlineid = relationsptr->Find(id_)->rightedge.ID;
        //左车道
        for(int j = 0; j < waysptr->Find(leftlineid)->length; ++j)
        {
            geometry_msgs::Point p_;
            p_.x = nodesptr->Find(waysptr->Find(leftlineid)->nodeline[j])->local_x;
            p_.y = nodesptr->Find(waysptr->Find(leftlineid)->nodeline[j])->local_y;
            p_.z = nodesptr->Find(waysptr->Find(leftlineid)->nodeline[j])->elevation;
            alane.leftpoints.push_back(p_);
        }
        //右车道
        for(int j = 0; j < waysptr->Find(rightlineid)->length; ++j)
        {
            geometry_msgs::Point p_;
            p_.x = nodesptr->Find(waysptr->Find(rightlineid)->nodeline[j])->local_x;
            p_.y = nodesptr->Find(waysptr->Find(rightlineid)->nodeline[j])->local_y;
            p_.z = nodesptr->Find(waysptr->Find(rightlineid)->nodeline[j])->elevation;
            alane.rightpoints.push_back(p_);
        }
        Lanesinfo.data.push_back(alane);
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

void HDMap::imuInit(const Eigen::Vector3d &imuMsg)
{
    if (++imucount < 101)
    {
        Eigen::Vector3d acc{imuMsg[0], imuMsg[1], imuMsg[2]};
        sum_acc += acc;
    }
    else if (!imuinit_flag)
    {
        avg_acc = sum_acc / 100.0;
        std::cout << "avg acc:  " << avg_acc.transpose().x() << "  " << avg_acc.transpose().y()
                  << "  " << avg_acc.transpose().z() << std::endl;
        double norm = avg_acc.norm();
        avg_acc /= norm;
        Eigen::Vector3d gravity_acc(Eigen::Vector3d(0, 0, 1));
        double cos_value = avg_acc.dot(gravity_acc);
        double angle = acos(cos_value);
        Eigen::Vector3d axis = GetSkewMatrix(avg_acc) * gravity_acc;
        // rot = Amatrix(angle * axis);
        Eigen::AngleAxisd rot_vec(angle, axis.normalized());
        rot = rot_vec.toRotationMatrix();
        std::cout << "rot: " << rot << std::endl;
        imuinit_flag = true;
    }
    else
    {
        Eigen::Vector3d acc{imuMsg[0], imuMsg[1], imuMsg[2]};
        adjusted_acc = rot * acc;
        std::cout << "adjust acc:  " << adjusted_acc.transpose().x() << "  " << adjusted_acc.transpose().y()
                  << "  " << adjusted_acc.transpose().z() << std::endl;
    }
}

void HDMap::outMapPlan(const centerway::CenterPoint3D &atnow_centerpoint, const double heading)
{
    int laneletid = centerwaysptr->findNearestLanelet(&atnow_centerpoint, heading);
    if(laneletid == -1)
    {
        ROS_WARN("can not plan!");
        return;
    }
    start_path = laneletid;
    start_centerpoint_id = globalplans->Atwhichpoint(atnow_centerpoint, centerwaysptr->Find(laneletid));
    if(isend_path_exist)
    {
        //visualmap->pathmarkerclear();
        paths.clear();
        paths = globalplans->run(start_path, end_path);
        if(!paths.empty())
        {
            visualmap->path2marker(centerwaysptr, paths, path_markerarray, currenttime);
            pushCenterPoint(paths);
            visualmap->smoothpath2marker(smoothpathnode, path_markerarray, currenttime);
            //发布导航信息
            fullNavigationInfo();
            navigation_pub.publish(laneletinfo);
            //发路径
            visualization_msgs::Marker golbalpath_ = path_markerarray.markers.back();
            golbalpath_pub.publish(golbalpath_);
            //std::cout << "path_markerarray size is " << path_markerarray.markers.size() << std::endl;
            //发下一个车道
            fullLanesInfo(start_path);
            lanes_pub.publish(Lanesinfo);
        }
    }else{
        ROS_WARN("goal point has not set!");
    } 
}

void HDMap::startpoint_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double xx = msg->pose.pose.position.x;
    double yy = msg->pose.pose.position.y;
    centerway::CenterPoint3D startpoint(xx, yy);
    int startpoint_res = globalplans->Inwhichcenterway(startpoint, nodesptr, waysptr, relationsptr);
    start_state[0] = xx;
    start_state[1] = yy;
    start_state[2] = tf::getYaw(msg->pose.pose.orientation);// [-pi,pi]
    //std::cout << "start yaw is " << start_state[2] << std::endl;
    ROS_INFO("start point is in %d path", startpoint_res);

    //暂时把起点的位置当作GPS定位信息test
    atnowpoint->local_x = xx;
    atnowpoint->local_y = yy;

    //plan
    if(startpoint_res != -1)
    {
        isstart_path_exist = true;
        start_path = startpoint_res;
        start_centerpoint_id = globalplans->Atwhichpoint(startpoint, centerwaysptr->Find(startpoint_res));
        ROS_INFO("start point is at %d", start_centerpoint_id);
        if(isend_path_exist)
        {
            //visualmap->pathmarkerclear();
            path_markerarray.markers.clear();
            paths.clear();
            paths = globalplans->run(start_path, end_path);
            if(!paths.empty())
            {
                visualmap->path2marker(centerwaysptr, paths, path_markerarray, currenttime);
                pushCenterPoint(paths);
                //可视化
                visualmap->smoothpath2marker(smoothpathnode, path_markerarray, currenttime);
                //发布导航信息
                fullNavigationInfo();
                navigation_pub.publish(laneletinfo);
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

void HDMap::goalpoint_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double xx = msg->pose.position.x;
    double yy = msg->pose.position.y;
    centerway::CenterPoint3D goalpoint(xx, yy);
    int goalpoint_res = globalplans->Inwhichcenterway(goalpoint, nodesptr, waysptr, relationsptr);
    end_state[0] = xx;
    end_state[1] = yy;
    end_state[2] = tf::getYaw(msg->pose.orientation);// [-pi,pi]
    endPose = msg->pose;
    ROS_INFO("goal point is in %d path", goalpoint_res);

    //plan
    if(goalpoint_res != -1)
    {
        //该点最近邻中心线点测试
        //int closestpointid = globalplans->Atwhichpoint(goalpoint, centerways->Find(goalpoint_res));
        //std::cout << "this point closest point id is: " << closestpointid << std::endl;
        
        //该点到边界距离测试
        // map::relation::relationship *relation_temp_ = relations->Find(goalpoint_res);
        // double leftdis = globalplans->Point2edgedistance(goalpoint, nodes, ways->Find(relation_temp_->leftedge.ID), goalpoint_res);
        // double rightdis = globalplans->Point2edgedistance(goalpoint, nodes, ways->Find(relation_temp_->rightedge.ID), goalpoint_res);
        // std::cout << "to left distance: " << leftdis << ", to right distence: " << rightdis << std::endl;
        
        isend_path_exist = true;
        end_path = goalpoint_res;
        end_centerpoint_id = globalplans->Atwhichpoint(goalpoint, centerwaysptr->Find(goalpoint_res));
        ROS_INFO("goal point is at %d", end_centerpoint_id);
        if(isstart_path_exist)
        {
            //visualmap->pathmarkerclear();
            path_markerarray.markers.clear();
            paths.clear();
            paths = globalplans->run(start_path, end_path);
            if(!paths.empty())
            {
                visualmap->path2marker(centerwaysptr, paths, path_markerarray, currenttime);
                pushCenterPoint(paths);
                visualmap->smoothpath2marker(smoothpathnode, path_markerarray, currenttime);

                //发布导航信息
                fullNavigationInfo();
                navigation_pub.publish(laneletinfo);

                //发路径
                visualization_msgs::Marker golbalpath_ = path_markerarray.markers.back();
                golbalpath_pub.publish(golbalpath_);
                
                //发下一个车道
                //fullLanesInfo(start_path);
                //lanes_pub.publish(Lanesinfo);
            }

            //该点到下一个路口(下一次转向)距离测试, 无视终点，直到在地图找到第一个转向路段s
            //double intersectiondis = centerways->length2intersection(start_centerpoint_id_, paths);
            //std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;
            
        }else{
            ROS_WARN("start point has not set!");
        }
    }else{
        ROS_WARN("goal point out of HD-map!");
    }
    
}

void HDMap::gps_callback(const fsd_common_msgs::Comb::ConstPtr &msg)
{
    atnowpoint->elevation = msg->Altitude;
    atnowpoint->latitude = msg->Lattitude;
    atnowpoint->longitude = msg->Longitude;

    vectormap->GPS2Localxy(atnowpoint);
    start_state[0] = atnowpoint->local_x;
    start_state[1] = atnowpoint->local_y;
    start_state[2] = (msg->Heading + 90)/180*M_PI;
    //imu矫正
    Eigen::Vector3d imuinmsg;
    imuinmsg << msg->Ax, -msg->Az, msg->Ay;
    imuInit(imuinmsg);
    //当前点定位到路段
    centerway::CenterPoint3D atnowcenterpoint = centerway::CenterPoint3D(*atnowpoint);
    //当前路段精确定位到某点
    //int atnowcenterway = globalplans->Inwhichcenterway(atnowcenterpoint, nodesptr, waysptr, relationsptr);
    std::vector<int> lanelets_res = globalplans->LocateLanelets(atnowcenterpoint, nodesptr, waysptr, relationsptr);
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
    path_markerarray.markers.clear();
    if(!lanelets_res.empty())
    {
        hdmapstate.inMap = true;

        //融合上一帧路径
        if(paths.empty())
        {
            //第一帧，优先选取第一个定位结果
            start_path = lanelets_res[0];
            std::cout << "first plan!" << std::endl;
        }else{
            //其他帧，优先选取上一帧路径lanelet范围内的结果
            bool isInLastPaths = false;
            for(int i = 0; i < lanelets_res.size(); i++)
            {
                if(std::find(paths.begin(), paths.end(), lanelets_res[i]) != paths.end())
                {
                    start_path = lanelets_res[i];
                    isInLastPaths = true;
                    std::cout << "select last frame lanelet id successed!" << std::endl;
                    break;
                }
            }
            //如果均不在上一帧lanelet范围内, 优先选取第一个
            if(!isInLastPaths) start_path = lanelets_res[0];
        }
        // start_path = atnowcenterway;
        
        hdmapstate.laneletid = start_path;
        start_centerpoint_id = globalplans->Atwhichpoint(atnowcenterpoint, centerwaysptr->Find(start_path));
        if(isend_path_exist)
        {
            hdmapstate.isEndExist = true;
            //visualmap->pathmarkerclear();
            paths.clear();
            paths = globalplans->run(start_path, end_path);
            if(!paths.empty())
            {
                hdmapstate.isFindPath = true;
                visualmap->path2marker(centerwaysptr, paths, path_markerarray, currenttime);
                pushCenterPoint(paths);
                visualmap->smoothpath2marker(smoothpathnode, path_markerarray, currenttime);
                //发布导航信息
                fullNavigationInfo();
                navigation_pub.publish(laneletinfo);
                //发路径
                visualization_msgs::Marker golbalpath_ = path_markerarray.markers.back();
                golbalpath_pub.publish(golbalpath_);
                //std::cout << "path_markerarray size is " << path_markerarray.markers.size() << std::endl;
                //发下一个车道
                fullLanesInfo(start_path);
                lanes_pub.publish(Lanesinfo);
            }else{
                smoothpathnode.clear();
                smoothpathnode.push_back(centerway::CenterPoint3D(*atnowpoint));
                double atnowpoint_heading = constrainAngle(msg->Heading)/180*M_PI;
                outMapPlan(atnowcenterpoint, atnowpoint_heading);
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
        smoothpathnode.clear();
        smoothpathnode.push_back(centerway::CenterPoint3D(*atnowpoint));
        double atnowpoint_heading = constrainAngle(msg->Heading)/180*M_PI;
        
        // int nearestcenterwayid = centerwaysptr->findNearestCenterwaypointid(atnowpoint);
        // centerway::CenterPoint3D tartgetpoint;
        // centerwaysptr->nextCenterwayPointid(nearestcenterwayid, tartgetpoint);
        // smoothpathnode.push_back(tartgetpoint);

        // visualmap->smoothpath2marker(smoothpathnode, path_markerarray, currenttime);
        // //发路径
        // visualization_msgs::Marker golbalpath_ = path_markerarray.markers.back();
        // golbalpath_pub.publish(golbalpath_);
        outMapPlan(atnowcenterpoint, atnowpoint_heading);
        
        ROS_WARN("gps point out of HD-map!");
    }

    //tf
    double headtemp = msg->Heading;
    headtemp = constrainAngle(headtemp);
    // std::cout << "headtemp: " << headtemp << std::endl;
    tf::Quaternion qenu2base;
    qenu2base.setRPY((msg->Roll)/180*M_PI, (msg->Pitch)/180*M_PI, (headtemp)/180*M_PI);
    baselink2map.setRotation(qenu2base);
    baselink2map.setOrigin(tf::Vector3(atnowpoint->local_x, atnowpoint->local_y, atnowpoint->elevation));
    //map->rslidar->base_link
    broadcaster.sendTransform(tf::StampedTransform(baselink2map, ros::Time::now(), "map", "rslidar"));
    std::cout << "atnowpoint update" << std::endl;
    std::cout << "now x: " << atnowpoint->local_x << ", now y: " << atnowpoint->local_y << ", now z: " << atnowpoint->elevation << std::endl;

    //Ve, Vn, Vu 转化为车辆坐标系速度
    Eigen::Matrix3d m3d;
    m3d << 1, 0, 0, 0, 0, -1, 0, -1, 0;
    Eigen::Quaterniond qbase2imu(m3d);
    Eigen::Quaterniond q1 = Eigen::Quaterniond(qenu2base.w(), qenu2base.x(), qenu2base.y(), qenu2base.z()).normalized();
    Eigen::Vector3d Venu = Eigen::Vector3d(msg->Ve, msg->Vn, msg->Vu);
    Eigen::Vector3d Vxyz = (q1*qbase2imu).normalized().inverse() * Venu;
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
    gpspath.header.frame_id = "map";
    gpspath.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = atnowpoint->local_x;
    pose_stamped.pose.position.y = atnowpoint->local_y;
    pose_stamped.pose.position.z = atnowpoint->elevation;
    pose_stamped.pose.orientation.x = qenu2base.x();
    pose_stamped.pose.orientation.y = qenu2base.y();
    pose_stamped.pose.orientation.z = qenu2base.z();
    pose_stamped.pose.orientation.w = qenu2base.w();
    gpspath.poses.push_back(pose_stamped);
    gpspath_pub.publish(gpspath);

    //car state
    hdmapstate.heading = headtemp;
    hdmapstate.carPose.position.x = atnowpoint->local_x;
    hdmapstate.carPose.position.y = atnowpoint->local_y;
    hdmapstate.carPose.position.z = atnowpoint->elevation;
    hdmapstate.carPose.orientation.x = qenu2base.x();
    hdmapstate.carPose.orientation.y = qenu2base.y();
    hdmapstate.carPose.orientation.z = qenu2base.z();
    hdmapstate.carPose.orientation.w = qenu2base.w();
    hdmapstate.endPose = endPose;
    hdmapstate.nextpoint.x = 0;
    hdmapstate.nextpoint.y = 0;
    hdmapstate.nextpoint.z = 0;
    if(smoothpathnode.size() >= 2) 
    {
        plan::Spline2D csp_obj(smoothpathnode);
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
    hdmapstate.velocity.x = Vxyz(0);
    hdmapstate.velocity.y = Vxyz(1);
    hdmapstate.velocity.z = Vxyz(2);
    hdmapstate.Accell.x = msg->Ax;
    hdmapstate.Accell.y = msg->Ay;
    hdmapstate.Accell.z = msg->Az;
    if(imuinit_flag)
    {
        hdmapstate.Accell.x = adjusted_acc[0];
        hdmapstate.Accell.y = adjusted_acc[1];
        hdmapstate.Accell.z = adjusted_acc[2];
    }
    carstate_pub.publish(hdmapstate);
}

HDMap::~HDMap()
{
    delete atnowpoint;
    delete visualmap;
    delete globalplans;
    delete vectormap;
}


} // namespace map
