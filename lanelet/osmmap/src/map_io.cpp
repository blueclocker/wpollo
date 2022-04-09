/*
 * @Author: your name
 * @Date: 2022-03-03 21:24:25
 * @LastEditTime: 2022-04-09 20:46:33
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/map_io.cpp
 */
#include "../include/osmmap/map_io.h"

namespace map
{

Map::Map(ros::NodeHandle n_):n(n_)
{
    n.getParam("file_path", file_path_);
    n.getParam("file_name", file_name_);
    //n.getParam("start_position", start_position_);
    //n.getParam("end_position", end_position_);
    navigation_pub = n.advertise<osmmap::navigation>("navigation_info", 1);

    std::string file = file_path_ + file_name_;
    TiXmlDocument doc;
    if(!doc.LoadFile(file.c_str()))
    {
        std::cout << "file input ERROR!" << std::endl;
        return;
    }
    TiXmlElement *ROOT = doc.FirstChildElement();
    if(ROOT == nullptr)
    {
        std::cout << "Failed to load file: No root element." << std::endl;
        doc.Clear();
        return;
    }
    node_pin = ROOT->FirstChildElement("node");
    way_pin = ROOT->FirstChildElement("way");
    relation_pin = ROOT->FirstChildElement("relation");

    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << std::endl;
    //node
    nodes = new node::Node(node_pin);
    nodes->CreateObjects(way_pin);
    std::cout << "nodes total number: " << nodes->Size() << std::endl;

    //way
    ways = new way::Way(way_pin);
    ways->CreateObjects(relation_pin);
    std::cout << "ways total number: " << ways->Size() << std::endl;
    //way::Line *pin = ways->Find(214);
    //std::cout << pin->Length() << std::endl;
    //if(pin == nullptr)
    //{
    //    std::cout << "not find" << std::endl;
    //}
    
    //relation
    relations = new relation::Relation(relation_pin);
    relations->CreateObjects(nullptr);
    std::cout << "relations total number: " << relations->Size() << std::endl;
    
    //centerway
    centerways = new centerway::CenterWay(nullptr);
    //centerways->Init();
    centerways->run(nodes, ways, relations);
    std::cout << "centerways total number: " << centerways->Size() << std::endl;
    std::cout << std::endl;
    
    //基准点设置，默认初始时不存在起点、终点
    isstart_path_exist = false;
    isend_path_exist = false;
    basicpoint = new node::Point3D(origin_lat, origin_lon, origin_ele);
    atnowpoint = new node::Point3D(0, 0);
    nodes->MercatorGPS2xy(basicpoint);
    std::cout << "origin has set already!" << std::endl;
    //std::cout << "origin x: " << basicpoint->local_x << ", origin y: " << basicpoint->local_y << std::endl;
    
    //output
    std::cout << "map init successful!!!" << std::endl;
    std::cout << std::endl;
    std::cout << "---------------------------------------------------" << std::endl;
    doc.Clear();
    
    //globalplan
    globalplans = new plan::Globalplan(centerways);
    //std::vector<int> paths = globalplans->run(start_position_, end_position_);

    //visualization_msgs::Marker
    visualmap = new MapVisualization(n);
    visualmap->map2marker(nodes, ways, centerways, relations);
    //visualmap->path2marker(centerways, paths);
    //visualmap->run(nodes, ways, centerways, relations);
    
    //main loop
    ros::Rate r(10);
    while(n.ok())
    {
        visualmap->run(nodes, ways, centerways, relations);
        startpoint_sub = n.subscribe("/initialpose", 10, &Map::startpoint_callback, this);
        goalpoint_sub = n.subscribe("/move_base_simple/goal", 10, &Map::goalpoint_callback, this);
        //gps_pub = n.subscribe("/gps/fix", 10, &Map::gps_callback, this);
        // navigation_pub.publish(laneletinfo);
        r.sleep();
        ros::spinOnce();
    }
}

void Map::Smoothpath(std::vector<int> pathid_)
{
    if(pathid_.empty()) return;
    //std::vector<centerway::CenterPoint3D> smoothpathnode;
    smoothpathnode.clear();
    //B样条插值
    for(int i = 0; i < pathid_.size(); ++i)
    {
        centerway::CenterWay3D *oneway = centerways->Find(pathid_[i]);
        
        bool flag_ = false;
        for(int j = 0; j < oneway->length - 1; ++j)
        {
            if(i == 0)
            {
                //第一段路
                if(oneway->centernodeline[j] == start_centerpoint_id_) flag_ = true;
                if(!flag_) continue;
                smoothpathnode.push_back(*centerways->Findcenterpoint(oneway->centernodeline[j]));
            }else if(i == pathid_.size() - 1){
                //最后一段路
                smoothpathnode.push_back(*centerways->Findcenterpoint(oneway->centernodeline[j]));
                if(oneway->centernodeline[j] == end_centerpoint_id_) break;
            }else{
                //中间
                smoothpathnode.push_back(*centerways->Findcenterpoint(oneway->centernodeline[j]));
            }
        }
    }
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

void Map::fullNavigationInfo()
{
    //计算起点与道路边界的距离
    centerway::CenterPoint3D startpoint_(atnowpoint->local_x, atnowpoint->local_y);
    map::relation::relationship *relation_temp_ = relations->Find(start_path_);
    double leftdis = globalplans->Point2edgedistance(startpoint_, nodes, ways->Find(relation_temp_->leftedge.ID), start_path_);
    double rightdis = globalplans->Point2edgedistance(startpoint_, nodes, ways->Find(relation_temp_->rightedge.ID), start_path_);
    //std::cout << "start point to left distance: " << leftdis << ", to right distence: " << rightdis << std::endl;

    //填充laneletinfo
    laneletinfo.CurrentSequenceIDs.clear();
    laneletinfo.TrafficSign.clear();

    laneletinfo.header.frame_id = "map";
    laneletinfo.header.stamp = ros::Time::now();
    laneletinfo.CurrentSequenceIDs.push_back(start_path_);
    laneletinfo.TargetSequeceIDs = start_path_;
    laneletinfo.SpeedLimits = relation_temp_->speed_limit;
    laneletinfo.ToLeftDistance = leftdis;
    laneletinfo.ToRightDistance = rightdis;
    laneletinfo.IntersectionDistance = centerways->length2intersection(start_centerpoint_id_, paths);
    
    std::vector<map::relation::regulatoryelement*> trafficsigninfo = relations->getRegulatoryelement(start_path_);
    for(int i = 0; i < trafficsigninfo.size(); ++i)
    {
        osmmap::regulatoryelement onesign;
        onesign.SignType = static_cast<int>(trafficsigninfo[i]->subtype);
        onesign.LaneletID = trafficsigninfo[i]->laneletid;
        onesign.CenterpointID = trafficsigninfo[i]->centerpoint3did;
        laneletinfo.TrafficSign.push_back(onesign);
    }
}

void Map::startpoint_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double xx = msg->pose.pose.position.x;
    double yy = msg->pose.pose.position.y;
    centerway::CenterPoint3D startpoint(xx, yy);
    int startpoint_res = globalplans->Inwhichcenterway(startpoint, nodes, ways, relations);
    ROS_INFO("start point is in %d path", startpoint_res);

    //暂时把起点的位置当作GPS定位信息test
    atnowpoint->local_x = xx;
    atnowpoint->local_y = yy;

    //plan
    if(startpoint_res != -1)
    {
        isstart_path_exist = true;
        start_path_ = startpoint_res;
        start_centerpoint_id_ = globalplans->Atwhichpoint(startpoint, centerways->Find(startpoint_res));
        ROS_INFO("start point is at %d", start_centerpoint_id_);
        if(isend_path_exist)
        {
            paths.clear();
            paths = globalplans->run(start_path_, end_path_);
            visualmap->path2marker(centerways, paths);
            Smoothpath(paths);
            //可视化
            visualmap->smoothpath2marker(smoothpathnode);

            //该点到下一个路口(下一次转向)距离测试
            //double intersectiondis = centerways->length2intersection(start_centerpoint_id_, paths);
            //std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;

            //发布导航信息
            fullNavigationInfo();
            navigation_pub.publish(laneletinfo);

        }else{
            ROS_WARN("goal point has not set!");
        } 
    }else{
        ROS_WARN("start point out of HD-map!");
    }
}

void Map::goalpoint_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double xx = msg->pose.position.x;
    double yy = msg->pose.position.y;
    centerway::CenterPoint3D goalpoint(xx, yy);
    int goalpoint_res = globalplans->Inwhichcenterway(goalpoint, nodes, ways, relations);
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
        end_path_ = goalpoint_res;
        end_centerpoint_id_ = globalplans->Atwhichpoint(goalpoint, centerways->Find(goalpoint_res));
        ROS_INFO("goal point is at %d", end_centerpoint_id_);
        if(isstart_path_exist)
        {
            paths.clear();
            paths = globalplans->run(start_path_, end_path_);
            visualmap->path2marker(centerways, paths);
            Smoothpath(paths);
            visualmap->smoothpath2marker(smoothpathnode);

            //该点到下一个路口(下一次转向)距离测试, 无视终点，直到在地图找到第一个转向路段s
            //double intersectiondis = centerways->length2intersection(start_centerpoint_id_, paths);
            //std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;

            //发布导航信息
            fullNavigationInfo();
            navigation_pub.publish(laneletinfo);
            
        }else{
            ROS_WARN("start point has not set!");
        }
    }else{
        ROS_WARN("goal point out of HD-map!");
    }
    
}

void Map::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    atnowpoint->elevation = msg->altitude;
    atnowpoint->latitude = msg->latitude;
    atnowpoint->longitude = msg->longitude;
    nodes->MercatorGPS2xy(atnowpoint);
    atnowpoint->elevation -= basicpoint->elevation;
    atnowpoint->local_x -= basicpoint->local_x;
    atnowpoint->local_y -= basicpoint->local_y; 

    //当前点定位到路段
    centerway::CenterPoint3D atnowcenterpoint = centerway::CenterPoint3D(*atnowpoint);
    //当前路段精确定位到某点
    int atnowcenterway = globalplans->Inwhichcenterway(atnowcenterpoint, nodes, ways, relations);
    //plan
    if(atnowcenterway != -1)
    {
        start_path_ = atnowcenterway;
        start_centerpoint_id_ = globalplans->Atwhichpoint(atnowcenterpoint, centerways->Find(atnowcenterway));
        if(isend_path_exist)
        {
            paths.clear();
            paths = globalplans->run(start_path_, end_path_);
            visualmap->path2marker(centerways, paths);
            Smoothpath(paths);
            visualmap->smoothpath2marker(smoothpathnode);

            //该点到下一个路口(下一次转向)距离测试
            double intersectiondis = centerways->length2intersection(start_centerpoint_id_, paths);
            std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;

            //发布导航信息
            fullNavigationInfo();
            navigation_pub.publish(laneletinfo);

        }else{
            ROS_WARN("goal point has not set!");
        } 
    }else{
        ROS_WARN("start point out of HD-map!");
    }

    //tf
    /*tf::Quaternion q;
    q.setRPY(0, 0, 0);
    baselink2map.setRotation(q);
    baselink2map.setOrigin(tf::Vector3(atnowpoint->local_x, atnowpoint->local_y, atnowpoint->elevation));
    broadcaster.sendTransform(tf::StampedTransform(baselink2map, ros::Time::now(), "map", "base_link"));
    std::cout << "atnowpoint update" << std::endl;
    std::cout << "now x: " << atnowpoint->local_x << ", now y: " << atnowpoint->local_y << ", now z: " << atnowpoint->elevation << std::endl;*/
}

Map::~Map()
{
    delete basicpoint;
    delete atnowpoint;
    delete nodes;
    delete ways;
    delete relations;
    delete visualmap;
    delete centerways;
    delete globalplans;
}


};//namespace map