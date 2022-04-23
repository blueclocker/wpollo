/*
 * @Author: your name
 * @Date: 2022-03-03 21:24:25
 * @LastEditTime: 2022-04-23 20:00:39
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/map_io.cpp
 */
#include "../include/osmmap/map_core.h"

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

    map_pub = n.advertise<visualization_msgs::MarkerArray>("map", 1);
    path_pub = n.advertise<visualization_msgs::MarkerArray>("path", 1);
    gpspath_pub = n.advertise<nav_msgs::Path>("gpspath_info", 1);
    navigation_pub = n.advertise<osmmap::navigation>("navigation_info", 1);
    carstate_pub = n.advertise<osmmap::carState>("carstate_info", 1);
    lanes_pub = n.advertise<osmmap::Lanes>("lanes_info", 1);

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

    //globalplan
    //若const对象想调用非const成员函数，则需要进行强制类型转换const_cast <T&>(Obj)
    //若const成员函数想调用非const成员函数，则需要对this指针进行强制类型转换const_cast <T&>(*this)
    globalplans = new plan::Globalplan(const_cast<centerway::CenterWay*>(centerwaysptr));

    //visualization_msgs::Marker
    visualmap = new MapVisualization();
    visualmap->map2marker(nodesptr, waysptr, centerwaysptr, relationsptr, map_markerarray, ros::Time::now());
    
    //main loop
    ros::Rate r(10);
    while(n.ok())
    {
        //visualmap->run();
        currenttime = ros::Time::now();
        map_pub.publish(map_markerarray);
        path_pub.publish(path_markerarray);
        startpoint_sub = n.subscribe("/initialpose", 10, &HDMap::startpoint_callback, this);
        goalpoint_sub = n.subscribe("/move_base_simple/goal", 10, &HDMap::goalpoint_callback, this);
        gps_sub = n.subscribe("/comb", 10, &HDMap::gps_callback, this);
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
        r.sleep();
        ros::spinOnce();
    }
}

void HDMap::Smoothpath(const std::vector<int> &pathid_)
{
    //std::vector<centerway::CenterPoint3D> smoothpathnode;
    smoothpathnode.clear();
    if(pathid_.empty()) return;
    //B样条插值
    if(pathid_.size() == 1)
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
    }
    
    /*for(int i = 0; i < pathid_.size(); ++i)
    {
        centerway::CenterWay3D *oneway = centerways->Find(pathid_[i]);
        
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
                    smoothpathnode.push_back(*centerways->Findcenterpoint(oneway->centernodeline[j]));
                    if(oneway->centernodeline[j] == end_centerpoint_id) break;
                }else{
                    smoothpathnode.push_back(*centerways->Findcenterpoint(oneway->centernodeline[j]));
                    if(oneway->centernodeline[j] == end_centerpoint_id) break;
                }
            }else if(i == 0){
                //第一段路
                if(oneway->centernodeline[j] == start_centerpoint_id) flag_ = true;
                if(!flag_) continue;
                smoothpathnode.push_back(*centerways->Findcenterpoint(oneway->centernodeline[j]));
            }else{
                //中间
                smoothpathnode.push_back(*centerways->Findcenterpoint(oneway->centernodeline[j]));
            }
        }
    }*/
    
    //std::cout << "smoothpathnode size: " << smoothpathnode.size() << std::endl;
    int *intnum = new int[smoothpathnode.size() - 1];
    for(int i = 0; i < smoothpathnode.size() - 1; ++i)
    {
        intnum[i] = 5;
    }
    int num2 = smoothpathnode.size();
    plan::CBSpline cbspline;
    cbspline.ThreeOrderBSplineInterpolatePt(smoothpathnode, num2, intnum);
    delete []intnum;
    //std::cout << "smoothpathnode after size: " << smoothpathnode.size() << std::endl;

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
    laneletinfo.direction = static_cast<int>(relation_temp_->turn_direction);
    laneletinfo.IntersectionDistance = centerwaysptr->length2intersection(start_centerpoint_id, paths, relationsptr);
    
    std::vector<map::relation::regulatoryelement*> trafficsigninfo = relationsptr->getRegulatoryelement(start_path);
    for(int i = 0; i < trafficsigninfo.size(); ++i)
    {
        osmmap::regulatoryelement onesign;
        onesign.SignType = static_cast<int>(trafficsigninfo[i]->subtype);
        onesign.LaneletID = trafficsigninfo[i]->laneletid;
        onesign.CenterpointID = trafficsigninfo[i]->centerpoint3did;
        laneletinfo.TrafficSign.push_back(onesign);
    }
}

void HDMap::fullLanesInfo(const int id_)
{
    Lanesinfo.header.frame_id = "map";
    Lanesinfo.header.stamp = currenttime;
}

void HDMap::startpoint_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double xx = msg->pose.pose.position.x;
    double yy = msg->pose.pose.position.y;
    centerway::CenterPoint3D startpoint(xx, yy);
    int startpoint_res = globalplans->Inwhichcenterway(startpoint, nodesptr, waysptr, relationsptr);
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

void HDMap::gps_callback(const fsd_common_msgs::comb::ConstPtr &msg)
{
    atnowpoint->elevation = msg->Altitude;
    atnowpoint->latitude = msg->Lattitude;
    atnowpoint->longitude = msg->Longitude;

    vectormap->GPS2Localxy(atnowpoint);
    //当前点定位到路段
    centerway::CenterPoint3D atnowcenterpoint = centerway::CenterPoint3D(*atnowpoint);
    //当前路段精确定位到某点
    int atnowcenterway = globalplans->Inwhichcenterway(atnowcenterpoint, nodesptr, waysptr, relationsptr);
    //HDmap state
    osmmap::carState hdmapstate;
    hdmapstate.header.frame_id = "map";
    hdmapstate.header.stamp = ros::Time::now();
    hdmapstate.inMap = false;
    hdmapstate.isEndExist = false;
    hdmapstate.isFindPath = false;
    hdmapstate.laneletid = 0;
    //plan
    //visualmap->pathmarkerclear();
    path_markerarray.markers.clear();
    if(atnowcenterway != -1)
    {
        hdmapstate.inMap = true;
        start_path = atnowcenterway;
        hdmapstate.laneletid = start_path;
        start_centerpoint_id = globalplans->Atwhichpoint(atnowcenterpoint, centerwaysptr->Find(atnowcenterway));
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
            }

            //该点到下一个路口(下一次转向)距离测试
            //double intersectiondis = centerways->length2intersection(start_centerpoint_id_, paths, relations);
            //std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;

        }else{
            ROS_WARN("goal point has not set!");
        } 
    }else{
        ROS_WARN("gps point out of HD-map!");
    }

    //tf
    tf::Quaternion q;
    q.setRPY((msg->Roll)/180*3.14159, (msg->Pitch)/180*3.14159, (msg->Heading + 90)/180*3.14159);
    baselink2map.setRotation(q);
    baselink2map.setOrigin(tf::Vector3(atnowpoint->local_x, atnowpoint->local_y, atnowpoint->elevation));
    //map->rslidar->base_link
    broadcaster.sendTransform(tf::StampedTransform(baselink2map, ros::Time::now(), "map", "rslidar"));
    std::cout << "atnowpoint update" << std::endl;
    std::cout << "now x: " << atnowpoint->local_x << ", now y: " << atnowpoint->local_y << ", now z: " << atnowpoint->elevation << std::endl;

    //gps path
    gpspath.header.frame_id = "map";
    gpspath.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = atnowpoint->local_x;
    pose_stamped.pose.position.y = atnowpoint->local_y;
    pose_stamped.pose.position.z = atnowpoint->elevation;
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    gpspath.poses.push_back(pose_stamped);
    gpspath_pub.publish(gpspath);

    //car state
    hdmapstate.carPose.position.x = atnowpoint->local_x;
    hdmapstate.carPose.position.y = atnowpoint->local_y;
    hdmapstate.carPose.position.z = atnowpoint->elevation;
    hdmapstate.carPose.orientation.x = q.x();
    hdmapstate.carPose.orientation.y = q.y();
    hdmapstate.carPose.orientation.z = q.z();
    hdmapstate.carPose.orientation.w = q.w();
    hdmapstate.velocity.x = msg->Ve;
    hdmapstate.velocity.y = msg->Vn;
    hdmapstate.velocity.z = msg->Vu;
    hdmapstate.Accell.x = msg->Ax;
    hdmapstate.Accell.y = msg->Ay;
    hdmapstate.Accell.z = msg->Az;
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