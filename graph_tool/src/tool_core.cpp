/*
 * @Author: your name
 * @Date: 2021-12-28 11:22:31
 * @LastEditTime: 2021-12-29 09:51:23
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/tool_core.cpp
 */
#include "graph_tool/tool_core.h"

GraphTool::GraphTool(ros::NodeHandle nh, const std::string file_path)
{
    map_path = file_path;
    map_path = map_path + "bit.xml";
    //key值
    key_name = {"x", "y", "angle", "angle", "distance", "angle"};
    key_type = {"double", "double", "double", "int", "double", "double"};
    key_for  = {"node", "node", "node", "node", "edge", "edge"};
    key_id   = {"d0", "d1", "d2", "d3", "d4", "d5"};

    //添加XML声明
    doc.LinkEndChild(new TiXmlDeclaration( "1.0", "utf-8", "" ));
    //添加根元素
    TiXmlElement *root = new TiXmlElement("graphml");
    doc.LinkEndChild(root);

    for(int i = 0; i < key_name.size(); ++i)
    {
        TiXmlElement* xmlkey = new TiXmlElement("key");
        root->LinkEndChild(xmlkey);
        xmlkey->SetAttribute("attr.name", key_name[i].c_str());
        xmlkey->SetAttribute("attr.type", key_type[i].c_str());
        xmlkey->SetAttribute("for", key_for[i].c_str());
        xmlkey->SetAttribute("id", key_id[i].c_str());
    }
    TiXmlElement* xmlgraph = new TiXmlElement("graph");
    root->LinkEndChild(xmlgraph);
    xmlgraph->SetAttribute("edgedefault", "directed");
    count = 0;
    linenumber = 0;
    head_write = xmlgraph;

    point_pub = n.advertise<visualization_msgs::MarkerArray>("points", 1);
    path_pub  = n.advertise<visualization_msgs::MarkerArray>("path", 1);
    line_sub = n.subscribe("/point_relationship",10, &GraphTool::path_callback, this);
    point_sub = n.subscribe("/initialpose", 10, &GraphTool::point_callback, this);

    ros::spin();
}

GraphTool::~GraphTool()
{
    doc.SaveFile(map_path.c_str());
    doc.Clear();
}

void GraphTool::point_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ros::Time time_at_now = ros::Time::now();
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    ROS_INFO("get point at x=%.2f, y=%.2f", x, y);

    TiXmlElement* xmlnode = new TiXmlElement("node");
    head_write->LinkEndChild(xmlnode);
    xmlnode->SetAttribute("id", count++);

    std::vector<double> tempdata = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    for(int i = 0; i < tempdata.size(); ++i)
    {
        TiXmlElement* xmldata = new TiXmlElement("data");
        xmlnode->LinkEndChild(xmldata);
        xmldata->SetAttribute("key", key_id[i].c_str());
        xmldata->LinkEndChild(new TiXmlText((std::to_string(tempdata[i])).c_str()));
    }
    ROS_INFO("map update");

    if(count % 100 == 0) doc.SaveFile(map_path.c_str());
    
    visualization_msgs::Marker pointinback;
    pointinback.header.stamp = time_at_now;
    pointinback.header.frame_id = "map";
    pointinback.ns = "point_visual";
    pointinback.action = visualization_msgs::Marker::ADD;
    pointinback.id = count;
    pointinback.type = visualization_msgs::Marker::POINTS;
    pointinback.scale.x = 0.1;
    pointinback.scale.y = 0.1;
    pointinback.color.r = 1.0;
    pointinback.color.a = 1.0;
    pointinback.pose.orientation.x = 0;
    pointinback.pose.orientation.y = 0;
    pointinback.pose.orientation.z = 0;
    pointinback.pose.orientation.w = 1;

    visualization_msgs::Marker pointtext;
    pointtext.header.stamp = time_at_now;
    pointtext.header.frame_id = "map";
    pointtext.ns = "point_text";
    pointtext.action = visualization_msgs::Marker::ADD;
    pointtext.id = count;
    pointtext.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    pointtext.scale.z = 0.2;
    pointtext.color.r = 1.0;
    pointtext.color.a = 1.0;
    pointtext.pose.position.x = x + 0.1;
    pointtext.pose.position.y = y + 0.1;
    pointtext.pose.position.z = 0;
    pointtext.pose.orientation.w = 1;
    pointtext.text = std::to_string(count - 1);

    geometry_msgs::Point node;
    node = msg->pose.pose.position;
    pointqueue.push_back(node);
    pointinback.points.push_back(node);
    points.markers.push_back(pointinback);
    points.markers.push_back(pointtext);
    point_pub.publish(points);
}

void GraphTool::path_callback(const graph_tool::PathPoint::ConstPtr &msg)
{
    connection(msg->path_source, msg->path_target);
}

void GraphTool::connection(int x_source, int y_target)
{
    TiXmlElement* xmledge = new TiXmlElement("edge");
    head_write->LinkEndChild(xmledge);
    xmledge->SetAttribute("source", x_source);
    xmledge->SetAttribute("target", y_target);

    double distance, angel;
    geometry_msgs::Point source_node, target_node;
    source_node = pointqueue[x_source];
    target_node = pointqueue[y_target];
    distance = std::sqrt((source_node.x-target_node.x) * (source_node.x-target_node.x) + 
                         (source_node.y-target_node.y) * (source_node.y-target_node.y));
    angel = std::atan((target_node.y-source_node.y)/(target_node.x-source_node.x))*180/3.14159;

    TiXmlElement* xmldatad4 = new TiXmlElement("data");
    xmledge->LinkEndChild(xmldatad4);
    xmldatad4->SetAttribute("key", key_id[4].c_str());
    xmldatad4->LinkEndChild(new TiXmlText(std::to_string(distance).c_str()));

    TiXmlElement* xmldatad5 = new TiXmlElement("data");
    xmledge->LinkEndChild(xmldatad5);
    xmldatad5->SetAttribute("key", key_id[5].c_str());
    xmldatad5->LinkEndChild(new TiXmlText(std::to_string(angel).c_str()));
    doc.SaveFile(map_path.c_str());

    visualization_msgs::Marker lineinback;
    lineinback.header.stamp = ros::Time::now();
    lineinback.header.frame_id = "map";
    lineinback.ns = "line_visual";
    lineinback.action = visualization_msgs::Marker::ADD;
    lineinback.id = linenumber++;
    lineinback.type = visualization_msgs::Marker::LINE_STRIP;
    lineinback.scale.x = 0.05;
    lineinback.color.g = 1.0;
    lineinback.color.a = 1.0;
    lineinback.pose.orientation.w = 1;
    lineinback.pose.orientation.x = 0;
    lineinback.pose.orientation.y = 0;
    lineinback.pose.orientation.z = 0;
    lineinback.points.push_back(source_node);
    lineinback.points.push_back(target_node);
    paths.markers.push_back(lineinback);
    path_pub.publish(paths);
}



