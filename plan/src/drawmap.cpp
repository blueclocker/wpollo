/*
 * @Author: your name
 * @Date: 2021-11-30 14:32:36
 * @LastEditTime: 2022-01-03 15:36:21
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/src/drawmap.cpp
 */
#include "plan/drawmap.h"

GlobalMap::GlobalMap(ros::NodeHandle &n_, const std::string file_path, const bool flag): n(n_), map_path(file_path), isread(flag)
{
    if(isread)
    {
        std::string map_name_ = "bit.xml";
        n_.getParam("map_name", map_name_);
        map_path = map_path + map_name_;
        //std::cout << map_path << std::endl;
        pub_globalmap = n.advertise<visualization_msgs::MarkerArray>("/global_map", 1);
        //pub_mapnode = n.advertise<visualization_msgs::MarkerArray>("/mappoint", 1);
        read_xml();
        drawmap();
        //std::cout << "publishing" << std::endl;
        while(n.ok())
        {
            map_publish();
            ros::Rate sleep_rate(10);
            sleep_rate.sleep();
        }
        //ros::spin();
    }else{
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

        //根元素下添加子元素1
        //std::cout << key_name.size() << std::endl;
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
        head_write = xmlgraph;
        //sub_globalmap = n.subscribe<geometry_msgs::PointStamped>("/map", 10, &GlobalMap::mapcallback, this);

        ROS_INFO("map has been wroten");
        sub_globalmap = n.subscribe<geometry_msgs::PointStamped>("/map", 10, &GlobalMap::mapcallback, this);
        ros::spin();
    }
}

GlobalMap::~GlobalMap()
{
    if(!isread)
    {
        doc.SaveFile(map_path.c_str());
        doc.Clear();
    }
}

void GlobalMap::read_xml()
{
    //xml文档
    TiXmlDocument doc;
    if(!doc.LoadFile(map_path.c_str()))
    {
        ROS_ERROR("file input error!");
        return;
    }
    //定义根节点变量并赋值为文档的第一个根节点
    TiXmlElement* root = doc.FirstChildElement();
    //如果没有找到根节点,说明是空XML文档或者非XML文档
    if(root == nullptr)
    {
        ROS_ERROR("Failed to load file: No root element.");
        //清理内存
        doc.Clear();
        return;
    }
    //遍历子节点
    /*for(auto iter = root->FirstChildElement(); iter != nullptr; iter=iter->NextSiblingElement())//不能用iter++
    {
        //获取元素名
        std::string elename = iter->Value();
        std::cout << elename << std::endl;
    }*/

    //
    TiXmlElement *graph_element = root->FirstChildElement("graph");
    TiXmlElement *node_element = graph_element->FirstChildElement("node");
    TiXmlElement *edge_element = graph_element->FirstChildElement("edge");

    for(auto it = node_element; it != edge_element; it=it->NextSiblingElement())
    {
        mapinfo tempinfo;
        tempinfo.id = std::atoi(it->Attribute("id"));
        for(auto iter = it->FirstChildElement(); iter != nullptr; iter=iter->NextSiblingElement())
        {
            std::string s(iter->Attribute("key"));
            if(s == "d0")
            {
                tempinfo.x = std::atof(iter->GetText());
                //std::cout << iter->GetText() << std::endl;
            }else if(s == "d1"){
                tempinfo.y = std::atof(iter->GetText());
            }else{
                //std::cout << "error" << std::endl;
                continue;
            }
        }
        //std::cout << tempinfo.id << std::endl;
        globalmap_info.push_back(tempinfo);
    }

    for(auto iter = edge_element; iter != nullptr; iter = iter->NextSiblingElement())
    {
        connect connectinfo;
        connectinfo.source = std::atoi(iter->Attribute("source"));
        connectinfo.target = std::atoi(iter->Attribute("target"));
        for(auto it = iter->FirstChildElement(); it != nullptr; it=it->NextSiblingElement())
        {
            std::string s(it->Attribute("key"));
            if(s == "d4")
            {
                connectinfo.distance = std::atof(it->GetText());
            }else if(s == "d5"){
                connectinfo.angle = std::atof(it->GetText());
            }else{
                //std::cout << "error" << std::endl;
                continue;
            }
        }
        connection.push_back(connectinfo);
    }
    //std::cout << connection.size() << std::endl;
    //std::cout << countedge << std::endl;
    ROS_INFO("map init successful");
    doc.Clear();
}

void GlobalMap::mapcallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    /*TiXmlDocument doc;
    if(!doc.LoadFile(map_path.c_str()))
    {
        ROS_ERROR("file input error!");
        return;
    }

    TiXmlElement* root = doc.FirstChildElement();
    if(root == nullptr)
    {
        ROS_ERROR("Failed to load file: No root element.");
        //清理内存
        doc.Clear();
        return;
    }*/
    
    //TiXmlElement* graph_element = root->FirstChildElement("graph");
    TiXmlElement* xmlnode = new TiXmlElement("node");
    head_write->LinkEndChild(xmlnode);
    xmlnode->SetAttribute("id", count++);

    std::vector<double> tempdata = {msg->point.x, msg->point.y, msg->point.z};
    for(int i = 0; i < tempdata.size(); ++i)
    {
        TiXmlElement* xmldata = new TiXmlElement("data");
        xmlnode->LinkEndChild(xmldata);
        xmldata->SetAttribute("key", key_id[i].c_str());
        xmldata->LinkEndChild(new TiXmlText((std::to_string(tempdata[i])).c_str()));
    }
    ROS_INFO("map update");

    if(count % 100 == 0) doc.SaveFile(map_path.c_str());
    //doc.SaveFile(map_path.c_str());
    //doc.Clear();
}

void GlobalMap::write_xml()
{
    TiXmlDocument doc;

    //添加XML声明
    doc.LinkEndChild(new TiXmlDeclaration( "1.0", "utf-8", "" ));
    //添加根元素
    TiXmlElement *root = new TiXmlElement("graphml");
    doc.LinkEndChild(root);

    //根元素下添加子元素1

    doc.SaveFile(map_path.c_str());
    doc.Clear();
}

void GlobalMap::drawmap()
{
    currenttime = ros::Time::now();
    for(int i = 0; i < globalmap_info.size(); ++i)
    {
        visualization_msgs::Marker point_one;
        point_one.header.stamp = currenttime;
        point_one.header.frame_id = "map";
        point_one.ns = "map_node";
        point_one.action = visualization_msgs::Marker::ADD;
        point_one.id = i;
        point_one.type = visualization_msgs::Marker::POINTS;
        point_one.scale.x = 2;
        point_one.scale.y = 2;
        point_one.color.r = 1.0;
        point_one.color.a = 1.0;
        point_one.pose.orientation.x = 0;
        point_one.pose.orientation.y = 0;
        point_one.pose.orientation.z = 0;
        point_one.pose.orientation.w = 1;

        geometry_msgs::Point node;
        node.x = globalmap_info[i].x;
        node.y = globalmap_info[i].y;
        node.z = 0;
        point_one.points.push_back(node);
        maps.markers.push_back(point_one);
        //
    }

    //visualization_msgs::MarkerArray maps;
    for(int i = 0; i < connection.size(); ++i)
    {
        visualization_msgs::Marker map_line;
        map_line.header.frame_id = "map";
        map_line.header.stamp = currenttime;
        map_line.ns = "map_graph";
        map_line.action = visualization_msgs::Marker::ADD;
        map_line.id = i;
        map_line.type = visualization_msgs::Marker::LINE_STRIP;
        map_line.scale.x = 0.1;
        map_line.color.g = 1.0;
        map_line.color.a = 1.0;
        map_line.pose.orientation.w = 1;
        map_line.pose.orientation.x = 0;
        map_line.pose.orientation.y = 0;
        map_line.pose.orientation.z = 0;

        geometry_msgs::Point sourcepoint, targetpoint;
        sourcepoint.x = globalmap_info[connection[i].source].x;
        sourcepoint.y = globalmap_info[connection[i].source].y;
        sourcepoint.z = 0;

        targetpoint.x = globalmap_info[connection[i].target].x;
        targetpoint.y = globalmap_info[connection[i].target].y;
        targetpoint.z = 0;

        map_line.points.push_back(sourcepoint);
        map_line.points.push_back(targetpoint);
        maps.markers.push_back(map_line);
    }
    //pub_globalmap.publish(maps);
    //std::cout << "publishing" << std::endl;
}

void GlobalMap::map_publish()
{
    pub_globalmap.publish(maps);
    //pub_globalmap.publish(points);
}

