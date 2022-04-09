/*
 * @Author: your name
 * @Date: 2021-12-28 11:22:31
 * @LastEditTime: 2022-03-22 15:36:37
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/tool_core.cpp
 */
#include "graph_tool/tool_core.h"

GraphTool::GraphTool(ros::NodeHandle nh)
{
    nh.getParam("map_path", map_path_);
    nh.getParam("map_name", map_name_);
    nh.getParam("isnew", isnew_);
    nh.getParam("searchradius", searchradius);
    map_path_save = map_path_ + map_name_ + "_" + std::to_string((int)(ros::Time::now()).toSec()) + ".xml";
    map_path_ = map_path_ + map_name_ + ".xml";
    count = 0;
    pathnumber = 0;
    iscatch = true;
    isaddpath = false;
    isdeletepoint = false;
    isdeletepath = false;
    two_node = true;
    //key值
    key_name = {"x", "y", "angle", "angle", "distance", "angle"};
    key_type = {"double", "double", "double", "int", "double", "double"};
    key_for  = {"node", "node", "node", "node", "edge", "edge"};
    key_id   = {"d0", "d1", "d2", "d3", "d4", "d5"};
    point_pub = n.advertise<visualization_msgs::MarkerArray>("points", 1);
    path_pub  = n.advertise<visualization_msgs::MarkerArray>("path", 1);
    f = boost::bind(&GraphTool::change_mode, this, _1);

    if(isnew_)
    {
        ROS_INFO("create a new xml map");

    }else{
        ROS_INFO("use an old map");
        if(!doc.LoadFile(map_path_.c_str()))
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
        TiXmlElement *graph_element = root->FirstChildElement("graph");
        //head_write = root->FirstChildElement("graph");
        TiXmlElement *node_element = graph_element->FirstChildElement("node");
        TiXmlElement *edge_element = graph_element->FirstChildElement("edge");

        for(auto it = node_element; it != edge_element; it=it->NextSiblingElement())
        {
            Node tempnode;
            tempnode.id_node = std::atoi(it->Attribute("id"));
            count = tempnode.id_node;
            for(auto iter = it->FirstChildElement(); iter != nullptr; iter=iter->NextSiblingElement())
            {
                std::string s(iter->Attribute("key"));
                if(s == "d0")
                {
                    tempnode.x_node = std::atof(iter->GetText());
                    //std::cout << iter->GetText() << std::endl;
                }else if(s == "d1"){
                    tempnode.y_node = std::atof(iter->GetText());
                }else{
                    //std::cout << "error" << std::endl;
                    continue;
                }
            }
            //std::cout << "node" << tempnode.id_node << std::endl;
            nodewithid.push_back(tempnode);
        }

        for(auto iter = edge_element; iter != nullptr; iter = iter->NextSiblingElement())
        {
            pathnumber++;
            Path connectinfo;
            connectinfo.source_path = std::atoi(iter->Attribute("source"));
            connectinfo.target_path = std::atoi(iter->Attribute("target"));
            for(auto it = iter->FirstChildElement(); it != nullptr; it=it->NextSiblingElement())
            {
                std::string s(it->Attribute("key"));
                if(s == "d4")
                {
                    connectinfo.distance_path = std::atof(it->GetText());
                    //std::cout << iter->GetText() << std::endl;
                }else{
                    connectinfo.angel_path = std::atof(it->GetText());
                }
            }
            pathwithpoints.push_back(connectinfo);
        }
        count++;
        pathnumber++;
        //std::cout << nodewithid.size() << std::endl;
        //std::cout << pathwithpoints.size() << std::endl;
        ROS_INFO("map init successful");
    }
    //ros::spin();
}

GraphTool::~GraphTool()
{
    //write_xml();
    doc.Clear();
}

void GraphTool::run()
{
    ros::Rate sleep_rate(10);
    while(n.ok())
    {
        drawmap();
        mode_server.setCallback(f);
        point_sub = n.subscribe("/initialpose", 10, &GraphTool::point_callback, this);
        sleep_rate.sleep();
        ros::spinOnce();
    }
}

void GraphTool::main_loop()
{
    ros::Rate sleep_rate(10);
    /*while(n.ok())
    {
        //point_pub = n.advertise<visualization_msgs::MarkerArray>("points", 1);
        //path_pub  = n.advertise<visualization_msgs::MarkerArray>("path", 1);
        drawmap();
        mode_server.setCallback(f);
        line_sub = n.subscribe("/point_relationship",10, &GraphTool::path_callback, this);
        point_sub = n.subscribe("/initialpose", 10, &GraphTool::point_callback, this);
        delete_point_sub = n.subscribe("/delete_point", 10, &GraphTool::delete_point, this);
        delete_line_sub = n.subscribe("/delete_path", 10, &GraphTool::delete_line, this);
        sleep_rate.sleep();
        ros::spinOnce();
        //ros::spin();
    }*/
}

void GraphTool::drawmap()
{
    points.markers.clear();
    paths.markers.clear();
    ros::Time currenttime = ros::Time::now();
    for(int i = 0; i < nodewithid.size(); ++i)
    {
        //std::cout << "drawnode" << std::endl;
        visualization_msgs::Marker point;
        point.header.stamp = currenttime;
        point.header.frame_id = "map";
        point.ns = "point_visual";
        point.action = visualization_msgs::Marker::ADD;
        point.id = i;
        point.type = visualization_msgs::Marker::POINTS;
        point.scale.x = 0.3;
        point.scale.y = 0.3;
        point.color.r = 1.0;
        point.color.a = 1.0;
        point.pose.orientation.x = 0;
        point.pose.orientation.y = 0;
        point.pose.orientation.z = 0;
        point.pose.orientation.w = 1;
        //point.frame_locked = false;
        point.lifetime = ros::Duration(0.5);

        visualization_msgs::Marker pointtext;
        pointtext.header.stamp = currenttime;
        pointtext.header.frame_id = "map";
        pointtext.ns = "point_text";
        pointtext.action = visualization_msgs::Marker::ADD;
        pointtext.id = i;
        pointtext.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        pointtext.scale.z = 0.3;
        pointtext.color.r = 1.0;
        pointtext.color.a = 1.0;
        pointtext.pose.position.x = nodewithid[i].x_node + 0.3;
        pointtext.pose.position.y = nodewithid[i].y_node + 0.3;
        pointtext.pose.position.z = 0;
        pointtext.pose.orientation.w = 1;
        //pointtext.frame_locked = false;
        pointtext.lifetime = ros::Duration(0.5);
        pointtext.text = std::to_string(nodewithid[i].id_node);

        geometry_msgs::Point node;
        node.x = nodewithid[i].x_node;
        node.y = nodewithid[i].y_node;
        node.z = 0;
        point.points.push_back(node);
        points.markers.push_back(point);
        points.markers.push_back(pointtext);
    }

    for(int i = 0; i < pathwithpoints.size(); ++i)
    {
        visualization_msgs::Marker map_line;
        map_line.header.frame_id = "map";
        map_line.header.stamp = currenttime;
        map_line.ns = "line_visual";
        map_line.action = visualization_msgs::Marker::ADD;
        map_line.id = i;
        map_line.type = visualization_msgs::Marker::LINE_STRIP;
        map_line.scale.x = 0.05;
        map_line.color.g = 1.0;
        map_line.color.a = 1.0;
        map_line.pose.orientation.w = 1;
        map_line.pose.orientation.x = 0;
        map_line.pose.orientation.y = 0;
        map_line.pose.orientation.z = 0;
        //map_line.frame_locked = false;
        map_line.lifetime = ros::Duration(0.5);

        geometry_msgs::Point sourcepoint, targetpoint;
        for(int j = 0; j < nodewithid.size(); j++)
        {
            if(pathwithpoints[i].source_path == nodewithid[j].id_node)
            {
                sourcepoint.x = nodewithid[j].x_node;
                sourcepoint.y = nodewithid[j].y_node;
                sourcepoint.z = 0;
            }

            if(pathwithpoints[i].target_path == nodewithid[j].id_node)
            {
                targetpoint.x = nodewithid[j].x_node;
                targetpoint.y = nodewithid[j].y_node;
                targetpoint.z = 0;
            }
        }

        map_line.points.push_back(sourcepoint);
        map_line.points.push_back(targetpoint);
        paths.markers.push_back(map_line);
    }
    //ROS_INFO("draw map");
    point_pub.publish(points);
    path_pub.publish(paths);
}

void GraphTool::change_mode(set_mode::modeConfig &config)
{
    iscatch = config.catch_point;
    isaddpath = config.add_path;
    isdeletepoint = config.delete_point;
    isdeletepath = config.delete_path;
    //ROS_INFO("mode updated");
    /*ROS_INFO("Reconfigure Request: %s %s %s %s",
        config.catch_point?"True":"False",
        config.add_path?"True":"False",
        config.delete_point?"True":"False",
        config.delete_path?"True":"False");*/
}

void GraphTool::point_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ros::Time time_at_now = ros::Time::now();
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    Node tempnode, getnode;
    tempnode.x_node = x;
    tempnode.y_node = y;
    if(getpoint(tempnode, getnode))
    {
        //nodewithid.push_back(getnode);
        ROS_INFO("catch an existing point%d at x=%.2f, y=%.2f", getnode.id_node, getnode.x_node, getnode.y_node);
        if(isaddpath)
        {
            ROS_INFO("add path mode ing...");
            //Node pathnodea, pathnodeb;
            if(two_node)
            {
                pathnodea = getnode;
                two_node = false;
                ROS_INFO("please select the next node");
            }else{
                pathnodeb = getnode;
                two_node = true;
                path_callback(pathnodea.id_node, pathnodeb.id_node);
            }
        }
        if(isdeletepoint)
        {
            ROS_INFO("delete point mode ing...");
            delete_point(getnode.id_node);
        }
        if(isdeletepath)
        {
            ROS_INFO("delete path mode ing...");
            if(two_node)
            {
                pathnodea = getnode;
                two_node = false;
                ROS_INFO("please select the next node");
            }else{
                pathnodeb = getnode;
                two_node = true;
                delete_path(pathnodea.id_node, pathnodeb.id_node);
            }
        }
    }else{
        tempnode.id_node = count++;
        nodewithid.push_back(tempnode);
        ROS_INFO("get a new point%d at x=%.2f, y=%.2f", count-1, x, y);
    }
    
}

void GraphTool::path_callback(const int x_source, const int y_target)
{
    //int x_source = msg->path_source;
    //int y_target = msg->path_target;
    for(int i = 0; i < pathwithpoints.size(); ++i)
    {
        if((x_source == pathwithpoints[i].source_path && y_target == pathwithpoints[i].target_path) 
        || (x_source == pathwithpoints[i].target_path && y_target == pathwithpoints[i].source_path))
        {
            ROS_ERROR("%d---%d path has aready existed", x_source, y_target);
            return;
        }
    }
    geometry_msgs::Point source_node, target_node;
    for(int i = 0; i < nodewithid.size(); i++)
    {
        if(nodewithid[i].id_node == x_source)
        {
            source_node.x = nodewithid[i].x_node;
            source_node.y = nodewithid[i].y_node;
            source_node.z = 0;
        }
        if(nodewithid[i].id_node == y_target)
        {
            target_node.x = nodewithid[i].x_node;
            target_node.y = nodewithid[i].y_node;
            target_node.z = 0;
        }
    }
    if(source_node == target_node || (source_node.x == 0 && source_node.y ==0) || (target_node.x == 0 &&target_node.y ==0))
    {
        ROS_ERROR("%d, %d node do not exist", x_source, y_target);
        return;
    }
    
    pathnumber++;
    double distance, angel;
    distance = std::sqrt((source_node.x-target_node.x) * (source_node.x-target_node.x) + 
                         (source_node.y-target_node.y) * (source_node.y-target_node.y));
    angel = std::atan((target_node.y-source_node.y)/(target_node.x-source_node.x))*180/3.14159;
    Path templine;
    templine.source_path = x_source;
    templine.target_path = y_target;
    templine.distance_path = distance;
    templine.angel_path = angel;
    pathwithpoints.push_back(templine);
    //drawmap();
    ROS_INFO("get path %d---%d", x_source, y_target);
}

void GraphTool::delete_point(const int msg)
{
    bool isfind = false;
    for(auto it = nodewithid.begin(); it != nodewithid.end();)
    {
        if(it->id_node == msg)
        {
            it = nodewithid.erase(it);
            isfind = true;
            break;
        }else{
            ++it;
        }
    }
    if(isfind)
    {
        if(msg == count-1) count--;
        for(auto it = pathwithpoints.begin(); it != pathwithpoints.end();)
        {
            if(it->source_path == msg || it->target_path == msg)
            {
                it = pathwithpoints.erase(it);
            }else{
                ++it;
            }
        }
        //drawmap();
        ROS_INFO("point %d and its paths have been deleted", msg);
    }else{
        ROS_ERROR("point %d do not exist", msg);
    }
}

void GraphTool::delete_path(const int x_source, const int y_target)
{
    for(auto it = pathwithpoints.begin(); it != pathwithpoints.end();)
    {
        if((it->source_path == x_source && it->target_path == y_target)
        || (it->source_path == y_target && it->target_path == x_source))
        {
            pathwithpoints.erase(it);
            //drawmap();
            ROS_INFO("path %d---%d has been delete", x_source, y_target);
            return;
        }else{
            ++it;
        }
    }
    ROS_ERROR("path %d---%d do not exist", x_source, y_target);
}

void GraphTool::write_xml()
{
    doc.Clear();
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
    head_write = xmlgraph;
    //xml写node
    for(int i = 0; i < nodewithid.size(); ++i)
    {
        TiXmlElement* xmlnode = new TiXmlElement("node");
        head_write->LinkEndChild(xmlnode);
        xmlnode->SetAttribute("id", nodewithid[i].id_node);

        std::vector<double> tempdata = {nodewithid[i].x_node, nodewithid[i].y_node, 0};
        for(int j = 0; j < tempdata.size(); ++j)
        {
            TiXmlElement* xmldata = new TiXmlElement("data");
            xmlnode->LinkEndChild(xmldata);
            xmldata->SetAttribute("key", key_id[j].c_str());
            xmldata->LinkEndChild(new TiXmlText((std::to_string(tempdata[j])).c_str()));
        }
        /*if(isnew_)
        {
            head_write->LinkEndChild(xmlnode);
        }else{
            head_write->InsertBeforeChild(head_write->FirstChildElement("edge"), *xmlnode);
        }*/
    }

    //xml写edge
    for(int i = 0; i < pathwithpoints.size(); ++i)
    {
        TiXmlElement* xmledge = new TiXmlElement("edge");
        head_write->LinkEndChild(xmledge);
        xmledge->SetAttribute("source", pathwithpoints[i].source_path);
        xmledge->SetAttribute("target", pathwithpoints[i].target_path);

        TiXmlElement* xmldatad4 = new TiXmlElement("data");
        xmledge->LinkEndChild(xmldatad4);
        xmldatad4->SetAttribute("key", key_id[4].c_str());
        xmldatad4->LinkEndChild(new TiXmlText(std::to_string(pathwithpoints[i].distance_path).c_str()));

        TiXmlElement* xmldatad5 = new TiXmlElement("data");
        xmledge->LinkEndChild(xmldatad5);
        xmldatad5->SetAttribute("key", key_id[5].c_str());
        xmldatad5->LinkEndChild(new TiXmlText(std::to_string(pathwithpoints[i].angel_path).c_str()));
    }
    doc.SaveFile(map_path_save.c_str());
    ROS_INFO("xml file has been saved");
}

bool GraphTool::getpoint(const Node &point_click, Node &pointback)
{
    Node tempnode;
    double maxdistance = 1000000.0;
    for(int i = 0; i < nodewithid.size(); ++i)
    {
        if(distance(nodewithid[i], point_click) < maxdistance)
        {
            tempnode = nodewithid[i];
            maxdistance = distance(nodewithid[i], point_click);
        }
    }
    if(maxdistance < searchradius)
    {
        pointback = tempnode;
        return true;
    }else{
        return false;
    }
}

double GraphTool::distance(Node pointa, Node pointb)
{
    double x = pointa.x_node - pointb.x_node;
    double y = pointa.y_node - pointb.y_node;
    return std::sqrt(x*x + y*y);
}



