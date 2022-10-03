/*
 * @Author: your name
 * @Date: 2022-03-05 17:50:11
 * @LastEditTime: 2022-10-03 19:07:34
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/hdmap/visualization.cpp
 */
#include "../include/osmmap/visualization.h"

namespace map
{

MapVisualization::MapVisualization()
{
    //map_pub = n.advertise<visualization_msgs::MarkerArray>("map", 1);
    //path_pub = n.advertise<visualization_msgs::MarkerArray>("path", 1);
    colors_ = new RGBcolor[5];
    colors_[0] = RGBcolor(1, 1, 1);
    colors_[1] = RGBcolor(1, 0, 0);
    colors_[2] = RGBcolor(0, 1, 0);
    colors_[3] = RGBcolor(0, 0.8, 1);
    colors_[4] = RGBcolor(0, 0, 0);
}

void MapVisualization::Nodes2Marker(const node::Point3D *pin, visualization_msgs::MarkerArray &map, ros::Time nowtime) const
{
    //node
    visualization_msgs::Marker markernode, markertxt;
    markernode.header.frame_id = "map";
    markernode.header.stamp = nowtime;
    markernode.ns = "edgepoints";
    markernode.action = visualization_msgs::Marker::ADD;
    markernode.id = pin->ID_;
    markernode.type = visualization_msgs::Marker::POINTS;
    markernode.scale.x = 0.3;
    markernode.scale.y = 0.3;
    markernode.color.r = colors_[static_cast<int>(species::NODE)].r_;//colors[static_cast<int>(species::NODE)].r
    markernode.color.g = colors_[static_cast<int>(species::NODE)].g_;
    markernode.color.b = colors_[static_cast<int>(species::NODE)].b_;
    markernode.color.a = 1.0;
    markernode.pose.orientation.x = 0;
    markernode.pose.orientation.y = 0;
    markernode.pose.orientation.z = 0;
    markernode.pose.orientation.w = 1;

    geometry_msgs::Point p;
    p.x = pin->local_x_;
    p.y = pin->local_y_;
    p.z = pin->elevation_;
    markernode.points.push_back(p);

    //txt
    markertxt.header.frame_id = "map";
    markertxt.header.stamp = nowtime;
    markertxt.ns = "edgepoints_number";
    markertxt.action = visualization_msgs::Marker::ADD;
    markertxt.id = pin->ID_;
    markertxt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markertxt.scale.z = 0.5;
    markertxt.color = markernode.color;
    markertxt.pose.position.x = p.x + 0.3;
    markertxt.pose.position.y = p.y + 0.3;
    markertxt.pose.position.z = p.z;
    markertxt.pose.orientation = markernode.pose.orientation;
    markertxt.text = std::to_string(pin->ID_);
    map.markers.push_back(markernode);
    map.markers.push_back(markertxt);
}

void MapVisualization::Ways2Marker(const node::Node *nodes, const way::Line *pin, visualization_msgs::MarkerArray &map, ros::Time nowtime) const
{
    //way
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = nowtime;
    marker.ns = "edge_ways";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = pin->ID_;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    //marker_.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    if(pin->type_ == way::WayType::road_border)
    {
        marker.scale.x = 0.1;
    }else{
        marker.scale.x = 0.05;
    }
    if(pin->type_ == way::WayType::line_thin || pin->type_ == way::WayType::road_border)
    {
        marker.color.r = colors_[static_cast<int>(species::EDGE)].r_;//colors[static_cast<int>(species::WAY)].r
        marker.color.g = colors_[static_cast<int>(species::EDGE)].g_;
        marker.color.b = colors_[static_cast<int>(species::EDGE)].b_;
    }else if(pin->type_ == way::WayType::stop_line){
        marker.color.r = colors_[static_cast<int>(species::STOP_LINE)].r_;//colors[static_cast<int>(species::WAY)].r
        marker.color.g = colors_[static_cast<int>(species::STOP_LINE)].g_;
        marker.color.b = colors_[static_cast<int>(species::STOP_LINE)].b_;
    }else{
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.1;
    }
    

    for(int i = 0; i < pin->length_; ++i)
    {
        geometry_msgs::Point p;
        node::Point3D *onenode = nodes->Find(pin->nodeline_[i]);
        p.x = onenode->local_x_;
        p.y = onenode->local_y_;
        p.z = onenode->elevation_;
        marker.points.push_back(p);
    }
    map.markers.push_back(marker);
}

void MapVisualization::Centerpoint2Marker(const centerway::CenterPoint3D *centerpoint, visualization_msgs::MarkerArray &map, ros::Time nowtime) const
{
    //centernode
    visualization_msgs::Marker marker, markertxt;
    marker.header.frame_id = "map";
    marker.header.stamp = nowtime;
    marker.ns = "centerpoints";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = centerpoint->ID_;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.color.r = colors_[static_cast<int>(species::NODE)].r_;
    marker.color.g = colors_[static_cast<int>(species::NODE)].g_;
    marker.color.b = colors_[static_cast<int>(species::NODE)].b_;
    marker.color.a = 1.0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    geometry_msgs::Point p;
    p.x = centerpoint->x_;
    p.y = centerpoint->y_;
    p.z = centerpoint->ele_;
    marker.points.push_back(p);

    //centertxt
    markertxt.header.frame_id = "map";
    markertxt.header.stamp = nowtime;
    markertxt.ns = "centerpoints_number";
    markertxt.action = visualization_msgs::Marker::ADD;
    markertxt.id = centerpoint->ID_;
    markertxt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markertxt.scale.z = 0.5;
    markertxt.color = marker.color;
    markertxt.pose.position.x = p.x + 0.3;
    markertxt.pose.position.y = p.y + 0.3;
    markertxt.pose.position.z = p.z;
    markertxt.pose.orientation = marker.pose.orientation;
    markertxt.text = std::to_string(centerpoint->ID_);
    map.markers.push_back(marker);
    map.markers.push_back(markertxt);
}

void MapVisualization::Centerway2Marker(const centerway::CenterWay *centerways, const centerway::CenterWay3D *centerway3ds, visualization_msgs::MarkerArray &map, ros::Time nowtime) const
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
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = nowtime;
    marker.ns = "center_ways";
    marker.action = visualization_msgs::Marker::ADD;
    //marker_.id = centerway3ds_->ID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.05;// 柄直径
    marker.scale.y = 0.5;// 箭头直径
    marker.scale.z = 0;// 长度,由于已经指定了起始点/终止点,这里不能再指定长度!!!
    marker.color.r = colors_[static_cast<int>(species::CENTER)].r_;//colors[static_cast<int>(species::WAY)].r
    marker.color.g = colors_[static_cast<int>(species::CENTER)].g_;
    marker.color.b = colors_[static_cast<int>(species::CENTER)].b_;
    marker.color.a = 0.8;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    for(int i = 0; i < centerway3ds->length_ - 1; ++i)
    {
        geometry_msgs::Point p, q;
        centerway::CenterPoint3D *pointa = centerways->FindCenterPoint(centerway3ds->centernodeline_[i]);
        centerway::CenterPoint3D *pointb = centerways->FindCenterPoint(centerway3ds->centernodeline_[i+1]);
        p.x = pointa->x_;
        p.y = pointa->y_;
        p.z = pointa->ele_;
        q.x = pointb->x_;
        q.y = pointb->y_;
        q.z = pointb->ele_;
        marker.id = pointa->ID_;
        marker.points.clear();
        //if(centerway3ds_->source == centerway3ds_->centernodeline[0])
        //{
        marker.points.push_back(p);
        marker.points.push_back(q);
        //}else{
            //marker_.points.push_back(q);
            //marker_.points.push_back(p);
        //}
        map.markers.push_back(marker);
    }
}

void MapVisualization::Path2Marker(const centerway::CenterWay *centerways, std::vector<int> paths, visualization_msgs::MarkerArray &path, ros::Time nowtime) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = nowtime;
    marker.ns = "plan_ways";
    marker.action = visualization_msgs::Marker::ADD;
    //marker_.id = centerway3ds_->ID;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 3;
    //marker_.scale.y = 0.5;
    //marker_.scale.z = 0;
    marker.color.r = colors_[static_cast<int>(species::PLAN_LINE)].r_;
    marker.color.g = colors_[static_cast<int>(species::PLAN_LINE)].g_;
    marker.color.b = colors_[static_cast<int>(species::PLAN_LINE)].b_;
    marker.color.a = 0.3;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.lifetime = ros::Duration(0.1);
    
    for(int i = 0; i < paths.size(); ++i)
    {
        centerway::CenterWay3D *away = centerways->Find(paths[i]);
        marker.id = away->ID_;
        marker.points.clear();
        for(int j = 0; j < away->length_; ++j)
        {
            geometry_msgs::Point p;
            centerway::CenterPoint3D *pointa = centerways->FindCenterPoint(away->centernodeline_[j]);
            //centerway::CenterPoint3D *pointb = centerways_->Findcenterpoint(away->centernodeline[j+1]);
            p.x = pointa->x_;
            p.y = pointa->y_;
            p.z = pointa->ele_;
            //q.x = pointb->x;
            //q.y = pointb->y;
            //q.z = pointb->ele;
            //marker_.id = away->ID;
            //marker_.points.clear();
            marker.points.push_back(p);
        }
        path.markers.push_back(marker);
    }
    //path.markers.clear();
    //path.markers.push_back(marker_);
}

void MapVisualization::Redgreenlight2Marker(const node::Node *nodes, const way::Way *ways, const relation::relationship *relation, visualization_msgs::MarkerArray &map, ros::Time nowtime) const
{
    way::Line *ref_line_line = ways->Find(relation->ref_line_);
    Ways2Marker(nodes, ref_line_line, map, nowtime);
    way::Line *light_bulbs_line = ways->Find(relation->light_bulbs_);
    way::Line *refers_line = ways->Find(relation->refers_);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = nowtime;
    marker.ns = "redgreenlight";//redgreenlight edgepoints
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    int *pin = light_bulbs_line->nodeline_;
    for(int i = 0; i < light_bulbs_line->length_; ++i)
    {
        //std::cout << i << " " << nodes_->Find(pin_[i])->ID << std::endl;
        marker.id = nodes->Find(pin[i])->ID_;
        if(i == 0) 
        {
            marker.color.r = 1.0;
            marker.color.g = 0;
            marker.color.b = 0;
        }
        if(i == 1) 
        {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0;
        }
        if(i == 2) 
        {
            marker.color.r = 0;
            marker.color.g = 1.0;
            marker.color.b = 0;
        }
        geometry_msgs::Point p;
        p.x = nodes->Find(pin[i])->local_x_;
        p.y = nodes->Find(pin[i])->local_y_;
        p.z = nodes->Find(pin[i])->elevation_;
        marker.points.clear();
        marker.points.push_back(p);
        map.markers.push_back(marker);
    }
    
    pin = refers_line->nodeline_;
    for(int i = 0; i < refers_line->length_; ++i)
    {
        marker.id = nodes->Find(pin[i])->ID_;
        marker.action = visualization_msgs::Marker::DELETE;
        geometry_msgs::Point p;
        p.x = nodes->Find(pin[i])->local_x_;
        p.y = nodes->Find(pin[i])->local_y_;
        p.z = nodes->Find(pin[i])->elevation_;
        marker.points.clear();
        marker.points.push_back(p);
        map.markers.push_back(marker);
    }
}

// Is angle AOB less than 180?
// https://qiita.com/fujii-kotaro/items/a411f2a45627ed2f156e
bool MapVisualization::IsAcuteAngle(const geometry_msgs::Point32 & a, const geometry_msgs::Point32 & o,
                                    const geometry_msgs::Point32 & b) const
{
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x) >= 0;
}

bool MapVisualization::IsWithinTriangle(const geometry_msgs::Point32 & a, const geometry_msgs::Point32 & b,
                                        const geometry_msgs::Point32 & c, const geometry_msgs::Point32 & p) const
{
    double c1 = (b.x - a.x) * (p.y - b.y) - (b.y - a.y) * (p.x - b.x);
    double c2 = (c.x - b.x) * (p.y - c.y) - (c.y - b.y) * (p.x - c.x);
    double c3 = (a.x - c.x) * (p.y - a.y) - (a.y - c.y) * (p.x - a.x);

    return c1 > 0.0 && c2 > 0.0 && c3 > 0.0 || c1 < 0.0 && c2 < 0.0 && c3 < 0.0;
}

void MapVisualization::AdjacentPoints(const int i, const int N, const geometry_msgs::Polygon poly, geometry_msgs::Point32 * p0,
                                      geometry_msgs::Point32 * p1, geometry_msgs::Point32 * p2) const
{
    if (p0 == nullptr || p1 == nullptr || p2 == nullptr) 
    {
        std::cout << "either p0, p1, or p2 is null pointer!" << std::endl;
        return;
    }

    *p1 = poly.points[i];
    if (i == 0)
        *p0 = poly.points[N - 1];
    else
        *p0 = poly.points[i - 1];

    if (i < N - 1)
        *p2 = poly.points[i + 1];
    else
        *p2 = poly.points[0];
}

void MapVisualization::Polygon2Triangle(const geometry_msgs::Polygon & polygon, std::vector<geometry_msgs::Polygon> * triangles) const
{
    geometry_msgs::Polygon poly = polygon;
    // ear clipping: find smallest internal angle in polygon
    int N = poly.points.size();

    // array of angles for each vertex
    std::vector<bool> is_acute_angle;
    is_acute_angle.assign(N, false);
    for (int i = 0; i < N; i++) 
    {
        geometry_msgs::Point32 p0, p1, p2;

        AdjacentPoints(i, N, poly, &p0, &p1, &p2);
        is_acute_angle.at(i) = IsAcuteAngle(p0, p1, p2);
    }

    // start ear clipping
    while (N >= 3) 
    {
        // std::cout << "N= " << N << std::endl;
        int clipped_vertex = -1;
        for (int i = 0; i < N; i++) 
        {
            // std::cout << "i= " << i << std::endl;
            bool theta = is_acute_angle.at(i);
            if (theta == true) 
            {
                geometry_msgs::Point32 p0, p1, p2;
                AdjacentPoints(i, N, poly, &p0, &p1, &p2);

                int j_begin = (i + 2) % N;
                int j_end = (i - 1 + N) % N;
                bool is_ear = true;
                for (int j = j_begin; j != j_end; j = (j + 1) % N) 
                {
                    if (IsWithinTriangle(p0, p1, p2, poly.points.at(j))) 
                    {
                        is_ear = false;
                        break;
                    }
                }

                if (is_ear) 
                {
                    clipped_vertex = i;
                    break;
                }
            }
        }
        if (clipped_vertex < 0 || clipped_vertex >= N) 
        {
            std::cout << "Could not find valid vertex for ear clipping triangulation. Triangulation result might be invalid" << std::endl;
            clipped_vertex = 0;
        }

        // create triangle
        geometry_msgs::Point32 p0, p1, p2;
        AdjacentPoints(clipped_vertex, N, poly, &p0, &p1, &p2);
        geometry_msgs::Polygon triangle;
        triangle.points.push_back(p0);
        triangle.points.push_back(p1);
        triangle.points.push_back(p2);
        triangles->push_back(triangle);

        // remove vertex of center of angle
        auto it = poly.points.begin();
        std::advance(it, clipped_vertex);
        poly.points.erase(it);

        // remove from angle list
        auto it_angle = is_acute_angle.begin();
        std::advance(it_angle, clipped_vertex);
        is_acute_angle.erase(it_angle);

        // update angle list
        N = poly.points.size();
        if (clipped_vertex == N) 
        {
            clipped_vertex = 0;
        }
        AdjacentPoints(clipped_vertex, N, poly, &p0, &p1, &p2);
        is_acute_angle.at(clipped_vertex) = IsAcuteAngle(p0, p1, p2);

        int i_prev = (clipped_vertex == 0) ? N - 1 : clipped_vertex - 1;
        AdjacentPoints(i_prev, N, poly, &p0, &p1, &p2);
        is_acute_angle.at(i_prev) = IsAcuteAngle(p0, p1, p2);
    }
}

void MapVisualization::Crosswalk2Marker(const node::Node *nodes, const way::Way *ways, const relation::relationship *crosswalk, visualization_msgs::MarkerArray &map, ros::Time nowtime) const
{
    // std::cout << crosswalk_->leftedge.ID << " " << crosswalk_->rightedge.ID << std::endl;
    auto leftway = ways->Find(crosswalk->leftedge_.ID_);
    auto rightway = ways->Find(crosswalk->rightedge_.ID_);

    geometry_msgs::Polygon cross;
    for(int i = 0; i < leftway->length_; ++i)
    {
        geometry_msgs::Point32 pt32;
        pt32.x = nodes->Find(leftway->nodeline_[i])->local_x_;
        pt32.y = nodes->Find(leftway->nodeline_[i])->local_y_;
        pt32.z = nodes->Find(leftway->nodeline_[i])->elevation_;
        cross.points.push_back(pt32);
    }
    for(int i = rightway->length_ - 1; i >= 0; --i)
    {
        geometry_msgs::Point32 pt32;
        pt32.x = nodes->Find(rightway->nodeline_[i])->local_x_;
        pt32.y = nodes->Find(rightway->nodeline_[i])->local_y_;
        pt32.z = nodes->Find(rightway->nodeline_[i])->elevation_;
        cross.points.push_back(pt32);
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = nowtime;
    // marker.frame_locked = true;
    marker.ns = "crosswalk";
    marker.id = crosswalk->ID_;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    // marker.lifetime = ros::Duration();
    marker.pose.position.x = 0.0;  // p.x();
    marker.pose.position.y = 0.0;  // p.y();
    marker.pose.position.z = 0.0;  // p.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    std_msgs::ColorRGBA c;
    c.r = 1.0;
    c.g = 0.5;
    c.b = 0.0;
    c.a = 0.3;
    std::vector<geometry_msgs::Polygon> triangles;
    Polygon2Triangle(cross, &triangles);
    for (int i = 0; i < triangles.size(); ++i) 
    {
        geometry_msgs::Point tri0[3];
        for (int j = 0; j < 3; j++) 
        {
            tri0[j].x = triangles[i].points[j].x;
            tri0[j].y = triangles[i].points[j].y;
            tri0[j].z = triangles[i].points[j].z;
            marker.points.push_back(tri0[j]);
            marker.colors.push_back(c);
        }
    }
    if (!marker.points.empty()) 
    {
        map.markers.push_back(marker);
    }
}

void MapVisualization::Smoothpath2Marker(const std::vector<map::centerway::CenterPoint3D> &smoothpath, visualization_msgs::MarkerArray &path, ros::Time nowtime) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = nowtime;
    marker.ns = "smooth_plan_ways";
    marker.action = visualization_msgs::Marker::ADD;
    //marker_.id = centerway3ds_->ID;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 1;
    //marker_.scale.y = 0.5;
    //marker_.scale.z = 0;
    marker.color.r = colors_[static_cast<int>(species::PLAN_LINE)].r_;
    marker.color.g = colors_[static_cast<int>(species::PLAN_LINE)].g_;
    marker.color.b = colors_[static_cast<int>(species::PLAN_LINE)].b_;
    marker.color.a = 0.6;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.lifetime = ros::Duration(0.1);
    
    for(int i = 0; i < smoothpath.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = smoothpath[i].x_;
        p.y = smoothpath[i].y_;
        p.z = smoothpath[i].ele_;
        marker.id = i;
        marker.points.push_back(p);
    }
    path.markers.push_back(marker);
}

void MapVisualization::Map2Marker(const node::Node *nodes, const way::Way *ways, const centerway::CenterWay *centerways, const relation::Relation *relations, visualization_msgs::MarkerArray &map, ros::Time nowtime) const
{
    //points
    for(auto it = nodes->Begin(); it != nodes->End(); ++it)
    {
        Nodes2Marker(it->second, map, nowtime);
    }

    //1st
    /*//ways
    for(auto it = ways_->Begin(); it != ways_->End(); ++it)
    {
        ways2marker(nodes_, it->second);
    }*/

    //2nd
    //ways, 由ralations指引
    for(auto it = relations->Begin(); it != relations->End(); ++it)
    {
        if(it->second->type_ == relation::RelationType::lanelet)
        {
            if(it->second->subtype_ == relation::RelationSubType::crosswalk)
            {
                Crosswalk2Marker(nodes, ways, it->second, map, nowtime);
                continue;
            }

            if(!ways->Find(it->second->leftedge_.ID_)->isVisual_)
            {
                Ways2Marker(nodes, ways->Find(it->second->leftedge_.ID_), map, nowtime);
                //ROS_INFO("leftedge id = %d", it->second->leftedge.ID);
                ways->Find(it->second->leftedge_.ID_)->isVisual_ = true;
            }

            if(!ways->Find(it->second->rightedge_.ID_)->isVisual_)
            {
                Ways2Marker(nodes, ways->Find(it->second->rightedge_.ID_), map, nowtime);
                ways->Find(it->second->rightedge_.ID_)->isVisual_ = true;
            }
            
        }else if(it->second->type_ == relation::RelationType::regulatory_element){
            if(it->second->subtype_ == relation::RelationSubType::traffic_light)
            {
                Redgreenlight2Marker(nodes, ways, it->second, map, nowtime);
            }else if(it->second->subtype_ == relation::RelationSubType::traffic_sign){
                Ways2Marker(nodes, ways->Find(it->second->ref_line_), map, nowtime);
            }
        }else{
            continue;
        }
    }
    //centerpoints
    for(auto it = centerways->CenterpointBegin(); it != centerways->CenterpointEnd(); ++it)
    {
        Centerpoint2Marker(it->second, map, nowtime);
    }
    //centerway
    for(auto it = centerways->Begin(); it != centerways->End(); ++it)
    {
        Centerway2Marker(centerways, it->second, map, nowtime);
        //std::cout << "centerway source: " << it->second->source << ", target: " << it->second->target << std::endl;
    }
}

MapVisualization::~MapVisualization()
{
    // std::cout << "~MapVisualization" << std::endl;
    delete colors_;
}


};//namespace map