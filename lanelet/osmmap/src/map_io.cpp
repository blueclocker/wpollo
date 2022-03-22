/*
 * @Author: your name
 * @Date: 2022-03-03 21:24:25
 * @LastEditTime: 2022-03-20 22:37:08
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
    isstart_position_exist = false;
    isend_position_exist = false;
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
        gps_pub = n.subscribe("/gps/fix", 10, &Map::gps_callback, this);
        r.sleep();
        ros::spinOnce();
    }
}

void Map::startpoint_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double xx = msg->pose.pose.position.x;
    double yy = msg->pose.pose.position.y;
    centerway::CenterPoint3D startpoint(xx, yy);
    int startpoint_res = globalplans->Inwhichcenterway(startpoint);
    ROS_INFO("start point is in %d path", startpoint_res);

    //plan
    if(startpoint_res != -1)
    {
        isstart_position_exist = true;
        start_position_ = startpoint_res;
        if(isend_position_exist)
        {
            std::vector<int> paths = globalplans->run(start_position_, end_position_);
            visualmap->path2marker(centerways, paths);
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
    int goalpoint_res = globalplans->Inwhichcenterway(goalpoint);
    ROS_INFO("goal point is in %d path", goalpoint_res);

    //plan
    if(goalpoint_res != -1)
    {
        isend_position_exist = true;
        end_position_ = goalpoint_res;
        if(isstart_position_exist)
        {
            std::vector<int> paths = globalplans->run(start_position_, end_position_);
            visualmap->path2marker(centerways, paths);
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