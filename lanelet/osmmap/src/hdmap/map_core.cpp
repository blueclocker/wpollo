/*
 * @Author: your name
 * @Date: 2022-03-03 21:24:25
 * @LastEditTime: 2022-09-12 14:27:58
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/hdmap/map_core.cpp
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
    
    //基准点设置，默认初始时不存在起点、终点
    isstart_path_exist = false;
    isend_path_exist = false;
    start_centerpoint_id = -1;
    end_centerpoint_id = -2;
    atnowpoint = new node::Point3D(0, 0);
    imucount = 0;
    imuinit_flag = false;

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

void HDMap::Smoothpath(const std::vector<int> &pathid_)
{
    //std::vector<centerway::CenterPoint3D> smoothpathnode;
    smoothpathnode.clear();
    if(pathid_.empty()) return;
    /*if(pathid_.size() == 1)
    {
        //只有一条路段, 肯定不换道
        centerway::CenterWay3D *oneway = centerwaysptr->Find(pathid_[0]);
        smoothpathnode.push_back(centerway::CenterPoint3D(*atnowpoint));
        
        bool flag_ = false;
        for(int j = 0; j < oneway->length; ++j)
        {
            if(oneway->centernodeline[j] == start_centerpoint_id) flag_ = true;
            if(!flag_) continue;
            smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
            if(oneway->centernodeline[j] == end_centerpoint_id) break;
        }
        smoothpathnode[0].ele = smoothpathnode[1].ele;
    }else if(pathid_.size() == 2){
        //两条路段，可能换道，可能不换
        centerway::CenterWay3D *oneway = centerwaysptr->Find(pathid_[0]);
        smoothpathnode.push_back(centerway::CenterPoint3D(*atnowpoint));
        if(centerwaysptr->isNeighbor(pathid_[0], pathid_[1]))
        {
            //换道
            if(start_centerpoint_id%100 >= end_centerpoint_id%100)
            {
                std::cout << "* maybe miss goal point, next turn... *" << std::endl;
                smoothpathnode.clear();
                return;
            }
            smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(start_centerpoint_id));
            oneway = centerwaysptr->Find(end_centerpoint_id/100);
            //借用相邻车道的中心线id基本一致，不严谨
            for(int j = start_centerpoint_id%100 + 1; j < oneway->length; ++j)
            {
                smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                if(oneway->centernodeline[j] == end_centerpoint_id) break;
            }
        }else{
            //不换道
            bool flag_ = false;
            for(int j = 0; j < oneway->length; ++j)
            {
                if(oneway->centernodeline[j] == start_centerpoint_id) flag_ = true;
                if(!flag_) continue;
                smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
            }
            oneway = centerwaysptr->Find(pathid_[1]);
            for(int j = 0; j < oneway->length; ++j)
            {
                smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                if(oneway->centernodeline[j] == end_centerpoint_id) break;
            }
        }
        smoothpathnode[0].ele = smoothpathnode[1].ele;
    }else{
        //三条及以上路段
        smoothpathnode.push_back(centerway::CenterPoint3D(*atnowpoint));
        for(int i = 0; i < pathid_.size() - 1; ++i)
        {
            centerway::CenterWay3D *oneway = centerwaysptr->Find(pathid_[i]);

            //如果换道
            if(centerwaysptr->isNeighbor(pathid_[i], pathid_[i+1]))
            {
                //i -> i+1 换道
                //i走到倒数第二点，i+1只保留最后一点
                bool flag_ = false;
                if(i == pathid_.size()-2)
                {
                    //最后一段路
                    smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[0]));
                    oneway = centerwaysptr->Find(pathid_[pathid_.size()-1]);
                    for(int j = 1; j < oneway->length; ++j)
                    {
                        smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                        if(oneway->centernodeline[j] == end_centerpoint_id) break;
                    }
                }else if(i == 0){
                    //第一段路
                    for(int j = 1; j < oneway->length - 1; ++j)
                    {
                        if(oneway->centernodeline[j] == start_centerpoint_id) flag_ = true;
                        if(!flag_) continue;
                        smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                    }
                    oneway = centerwaysptr->Find(pathid_[++i]);
                    smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[oneway->length-1]));
                }else{
                    //中间
                    for(int j = 1; j < oneway->length - 1; ++j)
                    {
                        smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                    }
                    oneway = centerwaysptr->Find(pathid_[++i]);
                    smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[oneway->length-1]));
                }
            }else{
                bool flag_ = false;
                for(int j = 1; j < oneway->length; ++j)
                {
                    if(i == 0)
                    {
                        //第一段路
                        if(oneway->centernodeline[j] == start_centerpoint_id) flag_ = true;
                        if(!flag_) continue;
                        smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                    }else{
                        //中间
                        smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                    }
                }
            }
        
        }
        
        smoothpathnode[0].ele = smoothpathnode[1].ele;
        //最后一条路段
        if(!centerwaysptr->isNeighbor(pathid_[pathid_.size()-1], pathid_[pathid_.size()-2]))
        {
            centerway::CenterWay3D *oneway = centerwaysptr->Find(pathid_[pathid_.size()-1]);
            for(int j = 0; j < oneway->length; ++j)
            {
                smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                if(oneway->centernodeline[j] == end_centerpoint_id) break;
            }
        }
    }*/
    
    smoothpathnode.push_back(centerway::CenterPoint3D(*atnowpoint));
    for(int i = 0; i < pathid_.size(); ++i)
    {
        centerway::CenterWay3D *oneway = centerwaysptr->Find(pathid_[i]);
        
        bool flag_ = false;
        for(int j = 0; j < oneway->length - 1; ++j)
        {
            if(i == pathid_.size() - 1)
            {
                //最后一段路
                if(i == 0)
                {
                    //同时也是第一段路
                    if(oneway->centernodeline[j] == start_centerpoint_id) flag_ = true;
                    if(!flag_) continue;
                    smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                    if(oneway->centernodeline[j] == end_centerpoint_id) break;
                }else{
                    smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
                    if(oneway->centernodeline[j] == end_centerpoint_id) break;
                }
            }else if(i == 0){
                //第一段路
                if(oneway->centernodeline[j] == start_centerpoint_id) flag_ = true;
                if(!flag_) continue;
                smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
            }else{
                //中间
                smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(oneway->centernodeline[j]));
            }
        }
    }
    smoothpathnode[0].ele = smoothpathnode[1].ele;
    smoothpathnode.erase(smoothpathnode.begin()+1);
    if(smoothpathnode.size() <= 3)
    {
        smoothpathnode.erase(smoothpathnode.begin()+1, smoothpathnode.end());
        smoothpathnode.push_back(centerway::CenterPoint3D(end_state[0], end_state[1]));
        smoothpathnode.back().ele = smoothpathnode[0].ele;
        // DubinsStateSpace db_planner(2.5);
        // std::vector<std::vector<double> > db_path;
        // double length_ = -1;
        // db_planner.sample(start_state, end_state, 0.2, length_, db_path);
        // std::vector<centerway::CenterPoint3D> smoothpathnodetemp;
        // for(int j = 0; j < db_path.size(); ++j)
        // {
        //     centerway::CenterPoint3D aa(db_path[j][0], db_path[j][1]);
        //     aa.ele = smoothpathnode[0].ele;
        //     smoothpathnodetemp.push_back(aa);
        // }
        // smoothpathnode.clear();
        // smoothpathnode = smoothpathnodetemp;
        return;
    }else{
        // smoothpathnode.erase(smoothpathnode.end()-2, smoothpathnode.end());
        smoothpathnode.erase(smoothpathnode.end());
        smoothpathnode.push_back(centerway::CenterPoint3D(end_state[0], end_state[1]));
        smoothpathnode.back().ele = smoothpathnode[1].ele;
    }

    //截取100米
    std::vector<centerway::CenterPoint3D> cutsmoothpathnode;
    double accumulatelength_ = 0;
    for(int i = 0; i < smoothpathnode.size(); ++i)
    {
        if(i == smoothpathnode.size() - 1)
        {
            cutsmoothpathnode.push_back(smoothpathnode[i]);
            break;
        }
        accumulatelength_ += centerwaysptr->NodeDistance(&smoothpathnode[i], &smoothpathnode[i+1]);
        if(accumulatelength_ > 100) break;
        cutsmoothpathnode.push_back(smoothpathnode[i]);
    }
    //std::cout << "publish golbal path length is " << accumulatelength_ << std::endl;
    smoothpathnode.clear();
    smoothpathnode = cutsmoothpathnode;
    
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
    // if(smoothpathnode.size() < 2) return;
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
    // smoothpathnode = temppathnode;
    //debug
    // double t = 0;
    // for(int i = 0; i < rcurvature.size(); ++i)
    // {
    //     if(std::fabs(rcurvature[i]) > t) t = std::fabs(rcurvature[i]);
        //std::cout << rcurvature[i] << " ";
    // }
    //std::cout << std::endl;
    // std::cout << "max rcurvature is " << t << std::endl;

    //dubins
    // double steer_radius = 2.5;//5.8
    // double step_size = 0.2;
    // std::vector<centerway::CenterPoint3D> smoothpathnodetemp;
    // //std::cout << "smoothpathnode size is " << smoothpathnode.size() << std::endl;
    // for(int i = 0; i < smoothpathnode.size(); ++i)
    // {
    //     DubinsStateSpace db_planner(steer_radius);
    //     std::vector<std::vector<double> > db_path;
    //     double length = -1;
    //     // if(i == smoothpathnode.size() - 2)//smoothpathnode.size() - 2
    //     // {
    //     //     double deltax = smoothpathnode[i].x - smoothpathnode[i - 1].x;
    //     //     double deltay = smoothpathnode[i].y - smoothpathnode[i - 1].y;
    //     //     double temp[3] = {smoothpathnode[i].x, smoothpathnode[i].y, atan2(deltay, deltax)};
    //     //     //std::cout << "angle1 is " << temp[2] << ", end angle2 is " << end_state[2] << std::endl;
    //     //     db_planner.sample(temp, end_state, step_size, length, db_path);
    //     // }else 
    //     if(i == 0){
    //         // double deltax = smoothpathnode[1].x - start_state[0];
    //         // double deltay = smoothpathnode[1].y - start_state[1];
    //         double deltax = smoothpathnode[i + 2].x - smoothpathnode[i + 1].x;
    //         double deltay = smoothpathnode[i + 2].y - smoothpathnode[i + 1].y;
    //         double temp[3] = {smoothpathnode[1].x, smoothpathnode[1].y, atan2(deltay, deltax)};
    //         //std::cout << "start angle1 is " << start_state[2] << ", angle2 is " << temp[2] << std::endl;
    //         db_planner.sample(start_state, temp, step_size, length, db_path);
    //     }else{
    //         double deltax = smoothpathnode[i].x - smoothpathnode[i - 1].x;
    //         double deltay = smoothpathnode[i].y - smoothpathnode[i - 1].y;
    //         double temp1[3] = {smoothpathnode[i].x, smoothpathnode[i].y, atan2(deltay, deltax)};
    //         deltax = smoothpathnode[i + 1].x - smoothpathnode[i].x;
    //         deltay = smoothpathnode[i + 1].y - smoothpathnode[i].y;
    //         double temp2[3] = {smoothpathnode[i+1].x, smoothpathnode[i+1].y, atan2(deltay, deltax)};
    //         //std::cout << "angle1 is " << temp1[2] << ", angle2 is " << temp2[2] << std::endl;
    //         if(temp2[0] == end_state[0] && temp2[1] == end_state[1]) temp2[2] = end_state[2];
    //         db_planner.sample(temp1, temp2, step_size, length, db_path);
    //     }
    //     for(int j = 0; j < db_path.size(); ++j)
    //     {
    //         centerway::CenterPoint3D aa(db_path[j][0], db_path[j][1]);
    //         aa.ele = smoothpathnode[i].ele;
    //         smoothpathnodetemp.push_back(aa);
    //     }
    // }
    // smoothpathnode.clear();
    // smoothpathnode = smoothpathnodetemp;
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
                Smoothpath(paths);
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
                Smoothpath(paths);
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
    //if(atnowcenterway != -1)
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
                Smoothpath(paths);
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
        smoothpathnode.push_back(*centerwaysptr->Findcenterpoint(centerwaysptr->returnMap(atnowpoint)));

        if(smoothpathnode.size() >= 2)
        {
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
            //debug
            // double t = 0;
            // for(int i = 0; i < rcurvature.size(); ++i)
            // {
            //     if(std::fabs(rcurvature[i]) > t) t = std::fabs(rcurvature[i]);
            //     //std::cout << rcurvature[i] << " ";
            // }
            // //std::cout << std::endl;
            // std::cout << "max rcurvature is " << t << std::endl;

            smoothpathnode = temppathnode;

            visualmap->smoothpath2marker(smoothpathnode, path_markerarray, currenttime);
            //发路径
            visualization_msgs::Marker golbalpath_ = path_markerarray.markers.back();
            golbalpath_pub.publish(golbalpath_);
        }
        ROS_WARN("gps point out of HD-map!");
    }

    //tf
    double headtemp = msg->Heading;
    headtemp = constrainAngle(headtemp);
    // std::cout << "headtemp: " << headtemp << std::endl;
    tf::Quaternion qenu2base;
    qenu2base.setRPY((msg->Roll)/180*3.14159, (msg->Pitch)/180*3.14159, (headtemp)/180*3.14159);
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


};//namespace map