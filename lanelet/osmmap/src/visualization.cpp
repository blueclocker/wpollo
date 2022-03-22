/*
 * @Author: your name
 * @Date: 2022-03-05 17:50:11
 * @LastEditTime: 2022-03-20 22:06:57
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/visualization.cpp
 */
#include "../include/osmmap/visualization.h"

namespace map
{

MapVisualization::MapVisualization(ros::NodeHandle &n_):n(n_)
{
    map_pub = n.advertise<visualization_msgs::MarkerArray>("map", 1);
    path_pub = n.advertise<visualization_msgs::MarkerArray>("path", 1);
    colors = new RGBcolor[5];
    colors[0] = RGBcolor(1, 1, 1);
    colors[1] = RGBcolor(1, 0, 0);
    colors[2] = RGBcolor(0, 1, 0);
    colors[3] = RGBcolor(0, 0, 1);
    colors[4] = RGBcolor(0, 0, 0);
}

void MapVisualization::nodes2marker(const node::Point3D *pin_)
{
    //node
    visualization_msgs::Marker markernode_, markertxt_;
    markernode_.header.frame_id = "/map";
    markernode_.header.stamp = currenttime;
    markernode_.ns = "edgepoints";
    markernode_.action = visualization_msgs::Marker::ADD;
    markernode_.id = pin_->ID;
    markernode_.type = visualization_msgs::Marker::POINTS;
    markernode_.scale.x = 0.3;
    markernode_.scale.y = 0.3;
    markernode_.color.r = colors[static_cast<int>(species::NODE)].r;//colors[static_cast<int>(species::NODE)].r
    markernode_.color.g = colors[static_cast<int>(species::NODE)].g;
    markernode_.color.b = colors[static_cast<int>(species::NODE)].b;
    markernode_.color.a = 1.0;
    markernode_.pose.orientation.x = 0;
    markernode_.pose.orientation.y = 0;
    markernode_.pose.orientation.z = 0;
    markernode_.pose.orientation.w = 1;

    geometry_msgs::Point p;
    p.x = pin_->local_x;
    p.y = pin_->local_y;
    p.z = pin_->elevation;
    markernode_.points.push_back(p);

    //txt
    markertxt_.header.frame_id = "/map";
    markertxt_.header.stamp = currenttime;
    markertxt_.ns = "edgepoints_number";
    markertxt_.action = visualization_msgs::Marker::ADD;
    markertxt_.id = pin_->ID;
    markertxt_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markertxt_.scale.z = 0.5;
    markertxt_.color = markernode_.color;
    markertxt_.pose.position.x = p.x + 0.3;
    markertxt_.pose.position.y = p.y + 0.3;
    markertxt_.pose.position.z = p.z;
    markertxt_.pose.orientation = markernode_.pose.orientation;
    markertxt_.text = std::to_string(pin_->ID);
    map.markers.push_back(markernode_);
    map.markers.push_back(markertxt_);
}

void MapVisualization::ways2marker(node::Node *nodes_, const way::Line *pin_)
{
    //way
    visualization_msgs::Marker marker_;
    marker_.header.frame_id = "/map";
    marker_.header.stamp = currenttime;
    marker_.ns = "edge_ways";
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.id = pin_->ID;
    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.scale.x = 0.1;
    marker_.color.a = 1.0;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 1;
    if(pin_->type == way::WayType::line_thin)
    {
        marker_.color.r = colors[static_cast<int>(species::EDGE)].r;//colors[static_cast<int>(species::WAY)].r
        marker_.color.g = colors[static_cast<int>(species::EDGE)].g;
        marker_.color.b = colors[static_cast<int>(species::EDGE)].b;
    }else if(pin_->type == way::WayType::stop_line){
        marker_.color.r = colors[static_cast<int>(species::STOP_LINE)].r;//colors[static_cast<int>(species::WAY)].r
        marker_.color.g = colors[static_cast<int>(species::STOP_LINE)].g;
        marker_.color.b = colors[static_cast<int>(species::STOP_LINE)].b;
    }else{
        marker_.color.r = 0;
        marker_.color.g = 0;
        marker_.color.b = 0;
        marker_.color.a = 0.1;
    }
    

    for(int i = 0; i < pin_->length; ++i)
    {
        geometry_msgs::Point p;
        node::Point3D *onenode = nodes_->Find(pin_->nodeline[i]);
        p.x = onenode->local_x;
        p.y = onenode->local_y;
        p.z = onenode->elevation;
        marker_.points.push_back(p);
    }
    map.markers.push_back(marker_);
}

void MapVisualization::centerpoint2marker(centerway::CenterPoint3D *centerpoint_)
{
    //centernode
    visualization_msgs::Marker marker_, markertxt_;
    marker_.header.frame_id = "/map";
    marker_.header.stamp = currenttime;
    marker_.ns = "centerpoints";
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.id = centerpoint_->ID;
    marker_.type = visualization_msgs::Marker::POINTS;
    marker_.scale.x = 0.3;
    marker_.scale.y = 0.3;
    marker_.color.r = colors[static_cast<int>(species::NODE)].r;
    marker_.color.g = colors[static_cast<int>(species::NODE)].g;
    marker_.color.b = colors[static_cast<int>(species::NODE)].b;
    marker_.color.a = 1.0;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 1;

    geometry_msgs::Point p;
    p.x = centerpoint_->x;
    p.y = centerpoint_->y;
    p.z = centerpoint_->ele;
    marker_.points.push_back(p);

    //centertxt
    markertxt_.header.frame_id = "/map";
    markertxt_.header.stamp = currenttime;
    markertxt_.ns = "centerpoints_number";
    markertxt_.action = visualization_msgs::Marker::ADD;
    markertxt_.id = centerpoint_->ID;
    markertxt_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markertxt_.scale.z = 0.5;
    markertxt_.color = marker_.color;
    markertxt_.pose.position.x = p.x + 0.3;
    markertxt_.pose.position.y = p.y + 0.3;
    markertxt_.pose.position.z = p.z;
    markertxt_.pose.orientation = marker_.pose.orientation;
    markertxt_.text = std::to_string(centerpoint_->ID);
    map.markers.push_back(marker_);
    map.markers.push_back(markertxt_);
}

void MapVisualization::centerway2marker(centerway::CenterWay *centerways_, centerway::CenterWay3D *centerway3ds_)
{
    //线带
    /*visualization_msgs::Marker marker_;
    marker_.header.frame_id = "/map";
    marker_.header.stamp = currenttime;
    marker_.ns = "center_ways";
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.id = centerway3ds_->ID;
    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.scale.x = 0.03;
    marker_.color.r = colors[static_cast<int>(species::EDGE)].r;//colors[static_cast<int>(species::WAY)].r
    marker_.color.g = colors[static_cast<int>(species::EDGE)].g;
    marker_.color.b = colors[static_cast<int>(species::EDGE)].b;
    marker_.color.a = 1.0;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 1;

    for(int i = 0; i < centerway3ds_->length; ++i)
    {
        geometry_msgs::Point p;
        centerway::CenterPoint3D *point = centerways_->Findcenterpoint(centerway3ds_->centernodeline[i]);
        p.x = point->x;
        p.y = point->y;
        p.z = point->ele;
        marker_.points.push_back(p);
    }*/

    //箭头
    visualization_msgs::Marker marker_;
    marker_.header.frame_id = "/map";
    marker_.header.stamp = currenttime;
    marker_.ns = "center_ways";
    marker_.action = visualization_msgs::Marker::ADD;
    //marker_.id = centerway3ds_->ID;
    marker_.type = visualization_msgs::Marker::ARROW;
    marker_.scale.x = 0.05;// 柄直径
    marker_.scale.y = 0.5;// 箭头直径
    marker_.scale.z = 0;// 长度,由于已经指定了起始点/终止点,这里不能再指定长度!!!
    marker_.color.r = colors[static_cast<int>(species::CENTER)].r;//colors[static_cast<int>(species::WAY)].r
    marker_.color.g = colors[static_cast<int>(species::CENTER)].g;
    marker_.color.b = colors[static_cast<int>(species::CENTER)].b;
    marker_.color.a = 1.0;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 1;

    for(int i = 0; i < centerway3ds_->length - 1; ++i)
    {
        geometry_msgs::Point p, q;
        centerway::CenterPoint3D *pointa = centerways_->Findcenterpoint(centerway3ds_->centernodeline[i]);
        centerway::CenterPoint3D *pointb = centerways_->Findcenterpoint(centerway3ds_->centernodeline[i+1]);
        p.x = pointa->x;
        p.y = pointa->y;
        p.z = pointa->ele;
        q.x = pointb->x;
        q.y = pointb->y;
        q.z = pointb->ele;
        marker_.id = pointa->ID;
        marker_.points.clear();
        if(centerway3ds_->source == centerway3ds_->centernodeline[0])
        {
            marker_.points.push_back(p);
            marker_.points.push_back(q);
        }else{
            marker_.points.push_back(q);
            marker_.points.push_back(p);
        }
        map.markers.push_back(marker_);
    }
}

void MapVisualization::path2marker(centerway::CenterWay *centerways_, std::vector<int> paths_)
{
    visualization_msgs::Marker marker_;
    marker_.header.frame_id = "/map";
    marker_.header.stamp = currenttime;
    marker_.ns = "plan_ways";
    marker_.action = visualization_msgs::Marker::ADD;
    //marker_.id = centerway3ds_->ID;
    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.scale.x = 3;
    //marker_.scale.y = 0.5;
    //marker_.scale.z = 0;
    marker_.color.r = colors[static_cast<int>(species::PLAN_LINE)].r;
    marker_.color.g = colors[static_cast<int>(species::PLAN_LINE)].g;
    marker_.color.b = colors[static_cast<int>(species::PLAN_LINE)].b;
    marker_.color.a = 0.5;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 1;
    marker_.lifetime = ros::Duration(0.1);
    
    for(int i = 0; i < paths_.size(); ++i)
    {
        centerway::CenterWay3D *away = centerways_->Find(paths_[i]);
        for(int j = 0; j < away->length; ++j)
        {
            geometry_msgs::Point p;
            centerway::CenterPoint3D *pointa = centerways_->Findcenterpoint(away->centernodeline[j]);
            //centerway::CenterPoint3D *pointb = centerways_->Findcenterpoint(away->centernodeline[j+1]);
            p.x = pointa->x;
            p.y = pointa->y;
            p.z = pointa->ele;
            //q.x = pointb->x;
            //q.y = pointb->y;
            //q.z = pointb->ele;
            marker_.id = pointa->ID;
            //marker_.points.clear();
            marker_.points.push_back(p);
            //marker_.points.push_back(q);
        }
    }
    path.markers.clear();
    path.markers.push_back(marker_);
}

void MapVisualization::redgreenlight2marker(node::Node *nodes_, way::Way *ways_, relation::relationship *relation_)
{
    way::Line *ref_line_line = ways_->Find(relation_->ref_line);
    ways2marker(nodes_, ref_line_line);
    way::Line *light_bulbs_line = ways_->Find(relation_->light_bulbs);
    way::Line *refers_line = ways_->Find(relation_->refers);
    visualization_msgs::Marker marker_;
    marker_.header.frame_id = "/map";
    marker_.header.stamp = currenttime;
    marker_.ns = "edgepoints";//redgreenlight
    marker_.action = visualization_msgs::Marker::MODIFY;
    marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_.scale.x = 0.3;
    marker_.scale.y = 0.3;
    marker_.scale.z = 0.3;
    marker_.color.a = 1.0;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 1;

    int *pin_ = light_bulbs_line->nodeline;
    for(int i = 0; i < light_bulbs_line->length; ++i)
    {
        //std::cout << i << " " << nodes_->Find(pin_[i])->ID << std::endl;
        marker_.id = nodes_->Find(pin_[i])->ID;
        if(i == 0) 
        {
            marker_.color.r = 1.0;
            marker_.color.g = 0;
            marker_.color.b = 0;
        }
        if(i == 1) 
        {
            marker_.color.r = 1.0;
            marker_.color.g = 1.0;
            marker_.color.b = 0;
        }
        if(i == 2) 
        {
            marker_.color.r = 0;
            marker_.color.g = 1.0;
            marker_.color.b = 0;
        }
        geometry_msgs::Point p;
        p.x = nodes_->Find(pin_[i])->local_x;
        p.y = nodes_->Find(pin_[i])->local_y;
        p.z = nodes_->Find(pin_[i])->elevation;
        marker_.points.clear();
        marker_.points.push_back(p);
        map.markers.push_back(marker_);
    }
    
    pin_ = refers_line->nodeline;
    for(int i = 0; i < refers_line->length; ++i)
    {
        marker_.id = nodes_->Find(pin_[i])->ID;
        marker_.action = visualization_msgs::Marker::DELETE;
        geometry_msgs::Point p;
        p.x = nodes_->Find(pin_[i])->local_x;
        p.y = nodes_->Find(pin_[i])->local_y;
        p.z = nodes_->Find(pin_[i])->elevation;
        marker_.points.clear();
        marker_.points.push_back(p);
        map.markers.push_back(marker_);
    }
}

void MapVisualization::map2marker(node::Node *nodes_, way::Way *ways_, centerway::CenterWay *centerways_, relation::Relation *relations_)
{
    currenttime = ros::Time::now();
    //1st
    //points
    /*for(auto it = nodes_->Begin(); it != nodes_->End(); ++it)
    {
        nodes2marker(it->second);
    }
    //ways
    for(auto it = ways_->Begin(); it != ways_->End(); ++it)
    {
        ways2marker(nodes_, it->second);
    }
    //centerpoints
    for(auto it = centerways_->centerpointBegin(); it != centerways_->centerpointEnd(); ++it)
    {
        centerpoint2marker(it->second);
    }
    //centerway
    for(auto it = centerways_->Begin(); it != centerways_->End(); ++it)
    {
        centerway2marker(centerways_, it->second);
    }*/

    //2nd
    //points
    for(auto it = nodes_->Begin(); it != nodes_->End(); ++it)
    {
        nodes2marker(it->second);
    }
    //ways, 由ralations指引
    for(auto it = relations_->Begin(); it != relations_->End(); ++it)
    {
        if(it->second->type == relation::RelationType::lanelet)
        {
            ways2marker(nodes_, ways_->Find(it->second->leftedge.ID));
            //ROS_INFO("leftedge id = %d", it->second->leftedge.ID);
            ways2marker(nodes_, ways_->Find(it->second->rightedge.ID));
        }else if(it->second->subtype == relation::RelationSubType::traffic_light){
            redgreenlight2marker(nodes_, ways_, it->second);
        }else{
            continue;
        }
    }
    //centerpoints
    for(auto it = centerways_->centerpointBegin(); it != centerways_->centerpointEnd(); ++it)
    {
        centerpoint2marker(it->second);
    }
    //centerway
    for(auto it = centerways_->Begin(); it != centerways_->End(); ++it)
    {
        centerway2marker(centerways_, it->second);
        //std::cout << "centerway source: " << it->second->source << ", target: " << it->second->target << std::endl;
    }
}

void MapVisualization::run(node::Node *nodes_, way::Way *ways_, centerway::CenterWay *centerways_, relation::Relation *relations_)
{
    //map2marker(nodes_, ways_, centerways_, relations_);
    //ros::Rate r(10);
    //while(n.ok())
    //{
        map_pub.publish(map);
        path_pub.publish(path);
        //r.sleep();
        //ros::spinOnce();
    //}
}

MapVisualization::~MapVisualization()
{
    //std::cout << "~MapVisualization" << std::endl;
    delete colors;
}


};//namespace map