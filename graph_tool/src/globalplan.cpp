/*
 * @Author: your name
 * @Date: 2022-01-11 15:15:35
 * @LastEditTime: 2022-03-13 14:21:19
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/globalplan.cpp
 */
#include "../include/graph_tool/globalplan.h"

GlobalPlan::GlobalPlan(ros::NodeHandle nh)
{
    nh.getParam("map_path", map_path_);
    nh.getParam("map_name", map_name_);
    nh.getParam("start_position", start_position_);
    nh.getParam("end_position", end_position_);
    map_path_ = map_path_ + map_name_ + ".xml";
    isfindpath = false;
    map_pub  = n.advertise<visualization_msgs::MarkerArray>("mapmarkers", 1);
    path_pub  = n.advertise<visualization_msgs::MarkerArray>("pathmarkers", 1);

    //读取地图
    TiXmlDocument doc;
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
    TiXmlElement *node_element = graph_element->FirstChildElement("node");
    TiXmlElement *edge_element = graph_element->FirstChildElement("edge");

    maxnodenumber = 0;
    for(auto it = node_element; it != edge_element; it=it->NextSiblingElement())
    {
        Node tempnode;
        tempnode.id_node = std::atoi(it->Attribute("id"));
        if(tempnode.id_node > maxnodenumber)
        {
            maxnodenumber = tempnode.id_node;
        }
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
    std::cout << "node total number: " << nodewithid.size() << std::endl;
    std::cout << "path total number: " << pathwithpoints.size() << std::endl;
    //std::cout << "maxnodenumber: " << maxnodenumber << std::endl;
    ROS_INFO("map init successful");
    doc.Clear();
}

GlobalPlan::~GlobalPlan()
{
    for(int i = 0; i < nodewithid.size(); ++i)
    {
        for(auto iter = map[i]; iter != nullptr; iter = iter->next)
        {
            delete iter;
        }
    }
}

Node GlobalPlan::findnode(const int id_)
{
    for(int i = 0; i < nodewithid.size(); ++i)
    {
        if(nodewithid[i].id_node == id_)
        {
            return nodewithid[i];
        }
    }
    return Node(-1);
}

double GlobalPlan::distance(const Node a, const Node b)
{
    double x = a.x_node - b.x_node;
    double y = a.y_node - b.y_node;
    return std::sqrt(x*x + y*y);
}

int GlobalPlan::findnext(const int id_, const int finalid)
{
    double distanceab = 100000.0;
    int nextid = -1;
    Node finalnode = map[finalid]->data;//findnode(finalid)
    for(auto iter = map[id_]->next; iter != nullptr; iter = iter->next)
    {
        if(!map[iter->data.id_node]->isvisit)
        {
            double distemp = distance(finalnode, iter->data);
            if(distemp < distanceab)
            {
                nextid = iter->data.id_node;
                distanceab = distemp;
            }
        }
    }
    //std::cout << "nextid:" << nextid << std::endl;
    return nextid;
}

void GlobalPlan::createmap()
{
    map = new mapnode* [std::max((int)(nodewithid.size()), maxnodenumber)+1];
    for(int i = 0; i <= std::max((int)(nodewithid.size()), maxnodenumber); ++i)
    {
        Node tempnode = findnode(i);
        if(tempnode.x_node == 0 && tempnode.y_node == 0)
        {
            map[i] = new mapnode(Node(-1));//头节点，第一个有效点在头节点之后第一个
        }else{
            map[i] = new mapnode(tempnode);
        }
        //ROS_INFO("init %d mapnode head", i);
    }

    Astarmap = new Astarmapnode[std::max((int)(nodewithid.size()), maxnodenumber)+1];
    for(int i = 0; i <= std::max((int)(nodewithid.size()), maxnodenumber); ++i)
    {
        Astarmap[i].parent = -1;
        Astarmap[i].id = i;
        Astarmap[i].F = DBL_MAX;
        Astarmap[i].G = DBL_MAX;
        Astarmap[i].H = DBL_MAX;
    }
    
    for(int i = 0; i < pathwithpoints.size(); ++i)
    {
        //无向图
        //source->target
        mapnode* nodesource = new mapnode(map[pathwithpoints[i].target_path]->data);//findnode(pathwithpoints[i].target_path)
        nodesource->next = map[pathwithpoints[i].source_path]->next;
        map[pathwithpoints[i].source_path]->next = nodesource;
        //target->source
        mapnode* nodetarget = new mapnode(map[pathwithpoints[i].source_path]->data);//findnode(pathwithpoints[i].source_path)
        nodetarget->next = map[pathwithpoints[i].target_path]->next;
        map[pathwithpoints[i].target_path]->next = nodetarget;
        //ROS_INFO("generate %d map", i);
    }
}

bool GlobalPlan::isnext(const int a, const int b)
{
    bool flag = false;
    for(auto it = map[b]->next; it != nullptr; it = it->next)
    {
        if(it->data.id_node == a)
        {
            flag = true;
            break;
        }
    }
    return flag;
}

void GlobalPlan::DFS(int x, int y)
{
    //DFS遍历
    /*for(int i = 0; i < nodewithid.size(); i++)
    {
        for(auto it = map[i]->next; it != nullptr; it = it->next)
        {
            //std::cout << it->data.id_node << " --- ";
            if(!map[it->data.id_node]->isvisit)
            {
                pathres.push_back(it->data.id_node);
                map[it->data.id_node]->isvisit = true;
            }
        }
        //std::cout << std::endl;
    }*/
    
    //DFS搜索
    if(pathres.empty())
    {
        pathres.push_back(x);
        map[x]->isvisit = true;
    }else{
        while(!isnext(pathres[pathres.size()-1], x))//删去回溯元素
        {
            pathres.pop_back();
        }
        pathres.push_back(x);
        map[x]->isvisit = true;
    }
    if(x == y)
    {
        ROS_INFO("find");
        return;
    }
    for(auto it = map[x]->next; it != nullptr; it = it->next)
    {
        if(!map[it->data.id_node]->isvisit)
        {
            DFS(it->data.id_node, y);
        }
        if(it->data.id_node == y) break;
    }
    return;

}

void GlobalPlan::DFS_pro(int x, int y)
{
    while(x != y)
    {
        //std::cout << "x: " << x << std::endl;
        map[x]->isvisit = true;
        pathres.push_back(x);
        //std::cout << "visit " << x << std::endl;
        int nextnodeid = findnext(x, y);
        if(nextnodeid == -1)
        {
            break;
        }else{
            x = nextnodeid;
        }
    }
    if(x == y)
    {
        pathres.push_back(x);
        ROS_INFO("find");
    }else{
        pathres.clear();
        ROS_ERROR("can not find");
    }
}

//从列表listx找到最小的总代价F
int GlobalPlan::findleastf(const std::list<int> &listx)
{
    if(listx.empty()) return -1;
    int idres = listx.front();
    for(auto it = listx.begin(); it != listx.end(); ++it)
    {
        //std::cout << Astarmap[*it].F << std::endl;
        if(Astarmap[*it].F < Astarmap[idres].F)
        {
            idres = *it;
        }
    }
    return idres;
}

//计算idg点到起点x的代价G
double GlobalPlan::calculateg(int x, int idg)
{
    if(Astarmap[idg].parent == -1)
    {
        return 0;
    }else{
        return Astarmap[Astarmap[idg].parent].G + distance(map[idg]->data, map[Astarmap[idg].parent]->data);
    }
}

//计算idh到终点y的代价H
double GlobalPlan::calculateh(int y, int idh)
{
    return std::abs(map[y]->data.x_node - map[idh]->data.x_node) + 
           std::abs(map[y]->data.y_node - map[idh]->data.y_node);
}

//计算idf点的总代价F
double GlobalPlan::calculatef(int idf)
{
    return Astarmap[idf].G + Astarmap[idf].H;
}

//寻找idx点的可行下一点
std::vector<int> GlobalPlan::getnextnode(int idx)
{
    std::vector<int> res;
    for(auto it = map[idx]->next; it != nullptr; it = it->next)
    {
        //如果该点不在关闭列表,把它加入待选点列表
        if(!isinlist(it->data.id_node, closelist))
        {
            res.push_back(it->data.id_node);
        }
    }
    return res;
}

//判断点x是否在列表listx中
bool GlobalPlan::isinlist(const int x, const std::list<int> &listx)
{
    bool resflag = false;
    for(auto it = listx.begin(); it != listx.end(); ++it)
    {
        if(*it == x)
        {
            resflag = true;
            break;
        }
    }
    return resflag;
}

//A*
void GlobalPlan::Astar(int x, int y)
{
    //std::list<int> openlist;
    //std::list<int> closelist;

    openlist.clear();
    closelist.clear();
    //放入起点
    openlist.push_back(x);
    Astarmap[x].G = 0;
    Astarmap[x].H = calculateh(y, x);
    Astarmap[x].F = calculatef(x);
    while (!openlist.empty())
    {
        int minidnode = findleastf(openlist);//找到F值最小的点

        openlist.remove(minidnode);//从开启列表中删除
        closelist.push_back(minidnode);//放到关闭列表

        //1,找到下一步待选点
        std::vector<int> candiates = getnextnode(minidnode);
        for(int i = 0; i < candiates.size(); ++i)
        {
            //2,对某一点，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
            if(!isinlist(candiates[i], openlist))
            {
                Astarmap[candiates[i]].parent = minidnode;
                Astarmap[candiates[i]].G = calculateg(x, candiates[i]);
                Astarmap[candiates[i]].H = calculateh(y, candiates[i]);
                Astarmap[candiates[i]].F = calculatef(candiates[i]);
                openlist.push_back(candiates[i]);
            }else{
                //3，对某一点，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
                double tempg = calculateg(x, candiates[i]);
                if(tempg > Astarmap[candiates[i]].G)
                {
                    continue;
                }else{
                    Astarmap[candiates[i]].parent = minidnode;
                    Astarmap[candiates[i]].G = tempg;
                    Astarmap[candiates[i]].F = calculatef(candiates[i]);
                }
            }
            //如果结束点出现在openlist则搜索成功
            if(isinlist(y, openlist))
            {
                Astarmap[y].parent = minidnode;
                ROS_INFO("find");
                int i = y;
                while(i != -1)
                {
                    pathres.push_back(i);
                    i = Astarmap[i].parent;
                }
                std::reverse(pathres.begin(), pathres.end());
                isfindpath = true;
                return;
            }
        }
    }
    ROS_INFO("not find");
    return;
}

//H恒等于0的A*
void GlobalPlan::Dijkstra(int x, int y)
{
    openlist.push_back(x);
    Astarmap[x].G = 0;
    Astarmap[x].H = 0;
    Astarmap[x].F = calculatef(x);
    while(!openlist.empty())
    {
        int minidnode = findleastf(openlist);

        openlist.remove(minidnode);
        closelist.push_back(minidnode);

        std::vector<int> candiates = getnextnode(minidnode);
        for(int i = 0; i < candiates.size(); ++i)
        {
            if(!isinlist(candiates[i], openlist))
            {
                Astarmap[candiates[i]].parent = minidnode;
                Astarmap[candiates[i]].G = calculateg(x, candiates[i]);
                Astarmap[candiates[i]].H = 0;
                Astarmap[candiates[i]].F = calculatef(candiates[i]);
                openlist.push_back(candiates[i]);
            }else{
                double tempg = calculateg(x, candiates[i]);
                if(tempg > Astarmap[candiates[i]].G)
                {
                    continue;
                }else{
                    Astarmap[candiates[i]].parent = minidnode;
                    Astarmap[candiates[i]].G = tempg;
                    Astarmap[candiates[i]].F = calculatef(candiates[i]);
                }
            }

            if(isinlist(y, openlist))
            {
                Astarmap[y].parent = minidnode;
                ROS_INFO("find");
                int i = y;
                while(i != -1)
                {
                    pathres.push_back(i);
                    i = Astarmap[i].parent;
                }
                std::reverse(pathres.begin(), pathres.end());
                isfindpath = true;
                return;
            }
        }
    }
    ROS_INFO("not find");
    return;
}

bool GlobalPlan::isturn(const Path &a, const Path &b)
{
    return (std::abs(a.angel_path - b.angel_path) > 10);
}

Path GlobalPlan::findpath(int a, int b)
{
    for(int i = 0; i < pathwithpoints.size(); ++i)
    {
        if(pathwithpoints[i].source_path == a && pathwithpoints[i].target_path == b)
        {
            return pathwithpoints[i];
        }
        if(pathwithpoints[i].target_path == a && pathwithpoints[i].source_path == b)
        {
            return pathwithpoints[i];
        }
    }

}

void GlobalPlan::findsmoothpath()
{
    smoothpathnode.clear();
    //找到拐弯系列点
    //1st 贝塞尔分段插值
    /*std::vector<Node> pathline, smoothpath;
    bool issmooth = false;
    for(int i = 0; i < pathres.size() - 2; ++i)
    {
        pathline.clear();
        smoothpath.clear();
        Path patha = findpath(pathres[i], pathres[i+1]);
        Path pathb = findpath(pathres[i+1], pathres[i+2]);
        if(isturn(patha, pathb))
        {
            pathline.push_back(map[pathres[i++]]->data);//
            pathline.push_back(map[pathres[i]]->data);//
            pathline.push_back(map[pathres[i+1]]->data);//findnode(pathres[i+1])
            Bezier bezier;
            smoothpath = bezier.CalculateSpline(pathline, 10);
            smoothpathnode.push_back(smoothpath);
            if((i + 1) == (pathres.size() - 1))
            {
                issmooth = true;
            }
        }else{
            pathline.push_back(map[pathres[i]]->data);//
            pathline.push_back(map[pathres[i+1]]->data);//
            smoothpathnode.push_back(pathline);
        }
    }
    if(!issmooth)
    {
        pathline.clear();
        pathline.push_back(map[pathres[pathres.size() - 2]]->data);//findnode(pathres[pathres.size() - 2])
        pathline.push_back(map[pathres[pathres.size() - 1]]->data);//findnode(pathres[pathres.size() - 1])
        smoothpathnode.push_back(pathline);
    }*/
    
    //2nd B样条插值
    //if(pathres.size() < 3) return;
    for(int i = 0; i < pathres.size(); ++i)
    {
        smoothpathnode.push_back(map[pathres[i]]->data);
    }
    int *intnum = new int[pathres.size() - 1];
    for(int i = 0; i < pathres.size() - 1; ++i)
    {
        intnum[i] = 5;
    }
    int num2 = pathres.size();
    CBSpline cbspline;
    cbspline.ThreeOrderBSplineInterpolatePt(smoothpathnode, num2, intnum);
    delete []intnum;

    //3rd 贝塞尔连续插值
    /*std::vector<Node> smoothpath;
    for(int i = 0; i < pathres.size(); ++i)
    {
        smoothpath.push_back(map[pathres[i]]->data);
    }
    Bezier bezier;
    smoothpathnode = bezier.CalculateSpline(smoothpath, 10);
    ROS_INFO("smoothpathnode length: %ld", smoothpathnode.size());*/
}

void GlobalPlan::pathvisual()
{
    map_marker.markers.clear();
    path_marker.markers.clear();
    ros::Time currenttime = ros::Time::now();
    for(int i = 0; i < nodewithid.size(); ++i)
    {
        //std::cout << "drawnode" << std::endl;
        visualization_msgs::Marker point;
        point.header.stamp = currenttime;
        point.header.frame_id = "/map";
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
        //point.lifetime = ros::Duration(0.5);

        visualization_msgs::Marker pointtext;
        pointtext.header.stamp = currenttime;
        pointtext.header.frame_id = "/map";
        pointtext.ns = "point_text";
        pointtext.action = visualization_msgs::Marker::ADD;
        pointtext.id = i;
        pointtext.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        pointtext.scale.z = 0.5;
        pointtext.color.r = 1.0;
        pointtext.color.a = 1.0;
        pointtext.pose.position.x = nodewithid[i].x_node + 0.3;
        pointtext.pose.position.y = nodewithid[i].y_node + 0.3;
        pointtext.pose.position.z = 0;
        pointtext.pose.orientation.w = 1;
        //pointtext.frame_locked = false;
        //pointtext.lifetime = ros::Duration(0.5);
        pointtext.text = std::to_string(nodewithid[i].id_node);

        geometry_msgs::Point node;
        node.x = nodewithid[i].x_node;
        node.y = nodewithid[i].y_node;
        node.z = 0;
        point.points.push_back(node);
        map_marker.markers.push_back(point);
        map_marker.markers.push_back(pointtext);
    }

    for(int i = 0; i < pathwithpoints.size(); ++i)
    {
        visualization_msgs::Marker map_line;
        map_line.header.frame_id = "/map";
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
        //map_line.lifetime = ros::Duration(0.5);

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
        map_marker.markers.push_back(map_line);
    }
    
    if(isfindpath)
    {
        for(int i = 0; i < pathres.size()-1; ++i)
        {
            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "/map";
            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = "findpath";
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = i;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.1;
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;
            line_strip.lifetime = ros::Duration(2);

            geometry_msgs::Point p, q;
            Node temp1 = map[pathres[i]]->data;//findnode(pathres[i])
            Node temp2 = map[pathres[i+1]]->data;//findnode(pathres[i+1])
            p.x = temp1.x_node;
            p.y = temp1.y_node;
            p.z = 0;
            q.x = temp2.x_node;
            q.y = temp2.y_node;
            q.z = 0;
            line_strip.points.push_back(p);
            line_strip.points.push_back(q);
            path_marker.markers.push_back(line_strip);
        }

        //路径平滑
        int nodenum = 0;
        for(int i = 0; i < smoothpathnode.size() - 1; ++i)
        {
            //for(int j = 0; j < smoothpathnode[i].size() - 1; ++j)
            //{
                visualization_msgs::Marker smoothpath_strip;
                smoothpath_strip.header.frame_id = "/map";
                smoothpath_strip.header.stamp = ros::Time::now();
                smoothpath_strip.ns = "smoothpath";
                smoothpath_strip.action = visualization_msgs::Marker::ADD;
                smoothpath_strip.pose.orientation.w = 1.0;
                smoothpath_strip.id = nodenum++;
                smoothpath_strip.type = visualization_msgs::Marker::LINE_STRIP;
                smoothpath_strip.scale.x = 0.1;
                smoothpath_strip.color.r = 1.0;
                smoothpath_strip.color.a = 1.0;
                smoothpath_strip.lifetime = ros::Duration(2);

                geometry_msgs::Point p, q;
                p.x = smoothpathnode[i].x_node;//smoothpathnode[i][j].x_node
                p.y = smoothpathnode[i].y_node;//smoothpathnode[i][j].y_node
                p.z = 0;
                q.x = smoothpathnode[i+1].x_node;//smoothpathnode[i][j+1].x_node
                q.y = smoothpathnode[i+1].y_node;//smoothpathnode[i][j+1].y_node
                q.z = 0;
                smoothpath_strip.points.push_back(p);
                smoothpath_strip.points.push_back(q);
                path_marker.markers.push_back(smoothpath_strip);
            //}
        }
    }
}

const std::vector<Node>& Bezier::CalculateSpline(
    const std::vector<Node>& control_points,
    const int& knot_number) 
{
    ctrl_num_ = control_points.size();
    control_points_ = control_points;
    knot_num_ = knot_number;
    //点数-1为阶数l
    order_ = control_points_.size() - 1;
    switch (order_) 
    {
        case 2:
            CalculateSecond();
            break;
        case 3:
            CalculateCubic();
            break;
        default:
            //std::cout << "No define order: " << order_ << std::endl;
            knot_num_ *= ctrl_num_;
            ROS_INFO("knot_num_: %d", knot_num_);
            Calculate(ctrl_num_);
            break;
    }
    return path_points_;
}

void Bezier::CalculateSecond() 
{
    if (ctrl_num_ != 3)
        return;
    path_points_.clear();
    double t = 0;
    double delta_t = 1.0 / (knot_num_);
    double f1, f2, f3;
    for (double i = 0; i <= knot_num_; ++i) 
    {
        t = delta_t * i;
        f1 = (1 - t) * (1 - t);
        f2 = 2 * (1 - t) * t;
        f3 = t * t;
        path_points_.push_back(
            Node(f1 * control_points_[0].x_node +
                 f2 * control_points_[1].x_node + f3 * control_points_[2].x_node,
                 f1 * control_points_[0].y_node + f2 * control_points_[1].y_node +
                 f3 * control_points_[2].y_node));
    }
}

void Bezier::CalculateCubic() 
{
    if (ctrl_num_ != 4)
        return;
    path_points_.clear();
    double t = 0;
    double delta_t = 1.0 / (knot_num_);
    double f1, f2, f3, f4;
    for (double i = 0; i <= knot_num_; ++i) 
    {
        t = delta_t * i;
        f1 = (1 - t) * (1 - t) * (1 - t);
        f2 = 3 * (1 - t) * (1 - t) * t;
        f3 = 3 * (1 - t) * t * t;
        f4 = t * t * t;
        path_points_.push_back(
            Node(f1 * control_points_[0].x_node + f2 * control_points_[1].x_node + 
                 f3 * control_points_[2].x_node + f4 * control_points_[3].x_node,
                 f1 * control_points_[0].y_node + f2 * control_points_[1].y_node +
                 f3 * control_points_[2].y_node + f4 * control_points_[3].y_node));
    }
}

void Bezier::Calculate(int Num) 
{
    path_points_.clear();
    double t = 0;
    double delta_t = 1.0 / (knot_num_);
    for(double i = 0; i <= knot_num_; ++i)
    {
        t = delta_t * i;
        double x = 0, y = 0, f;
        for(int j = 0; j < Num; ++j)
        {
            f = C_nm(Num - 1, j) * std::pow(t, j) * std::pow(1-t, Num - 1 - j);
            x += f * control_points_[j].x_node;
            y += f * control_points_[j].y_node;
        }
        path_points_.push_back(Node(x, y));
    }
}


//C(n,m)=n!/((n-m)!*m!), m <= n
int Bezier::C_nm(int n, int m)
{
    if(m == 0 || n == m) return 1;
    int sum = 1;
    for(int i = n; i > n - m; --i)
    {
        sum *= i;
    }
    for(int i = 1; i <= m; ++i)
    {
        sum /= i;
    }
    return sum;
}

CBSpline::CBSpline()
{
}
 
 
CBSpline::~CBSpline()
{
}
//======================================================================
// 函数功能： 二次B样条平滑，把给定的点，平滑到B样条曲线上，不增加点的数目
// 输入参数： *pt ：给定点序列，执行完成后，会被替换成新的平滑点
//            Num：点个数
// 返回值：   无返回值
//======================================================================
void CBSpline::TwoOrderBSplineSmooth(std::vector<Node> &pt, int Num)
{
	Node *temp = new Node[Num];
	for(int i = 0; i < Num; i++)
		temp[i] = pt[i];
 
	temp[0].x_node = 2*temp[0].x_node - temp[1].x_node;// 将折线两端点换成延长线上两点
	temp[0].y_node = 2*temp[0].y_node - temp[1].y_node;
 
	temp[Num-1].x_node = 2*temp[Num-1].x_node - temp[Num-2].x_node;
	temp[Num-1].y_node = 2*temp[Num-1].y_node - temp[Num-2].y_node;
 
	Node NodePt1, NodePt2, NodePt3;
	double t;
    for(int i = 0; i < Num - 2; i++)
	{
		NodePt1 = temp[i]; 
        NodePt2 = temp[i+1]; 
        NodePt3 = temp[i+2];
		if(i == 0){ // 第一段取t=0和t=0.5点   
			t = 0;
			pt[i].x_node = F02(t)*NodePt1.x_node + F12(t)*NodePt2.x_node + F22(t)*NodePt3.x_node;
			pt[i].y_node = F02(t)*NodePt1.y_node + F12(t)*NodePt2.y_node + F22(t)*NodePt3.y_node;
			t = 0.5;
			pt[i+1].x_node = F02(t)*NodePt1.x_node + F12(t)*NodePt2.x_node + F22(t)*NodePt3.x_node;
			pt[i+1].y_node = F02(t)*NodePt1.y_node + F12(t)*NodePt2.y_node + F22(t)*NodePt3.y_node;
		}else if(i == Num - 3){ // 最后一段取t=0.5和t=1点
			t = 0.5;
			pt[i+1].x_node = F02(t)*NodePt1.x_node + F12(t)*NodePt2.x_node + F22(t)*NodePt3.x_node;
			pt[i+1].y_node = F02(t)*NodePt1.y_node + F12(t)*NodePt2.y_node + F22(t)*NodePt3.y_node;
			t = 1;
			pt[i+2].x_node = F02(t)*NodePt1.x_node + F12(t)*NodePt2.x_node + F22(t)*NodePt3.x_node;
			pt[i+2].y_node = F02(t)*NodePt1.y_node + F12(t)*NodePt2.y_node + F22(t)*NodePt3.y_node;
		}else{ // 中间段取t=0.5点
			t = 0.5;
			pt[i+1].x_node = F02(t)*NodePt1.x_node + F12(t)*NodePt2.x_node + F22(t)*NodePt3.x_node;
			pt[i+1].y_node = F02(t)*NodePt1.y_node + F12(t)*NodePt2.y_node + F22(t)*NodePt3.y_node;
		}
	}
	delete []temp;
}
 
//================================================================
// 函数功能： 二次B样条拟合,在节点之间均匀插入指定个数点
// 输入参数： *pt ：给定点序列，执行完成后，会被替换成新的数据点
//            Num：节点点个数
//            *InsertNum: 节点之间需要插入的点个数指针 
// 返回值：   无返回值
//
//=================================================================
void CBSpline::TwoOrderBSplineInterpolatePt(std::vector<Node> &pt, int Num, int *InsertNum)
{
	if(pt.size() < 2 || InsertNum == NULL)
    {
        return;
    }
 
	int InsertNumSum = 0; // 计算需要插入的点总数
	for(int i = 0; i < Num - 1; i++)  InsertNumSum += InsertNum[i];
 
	Node *temp = new Node[Num]; // 二次B样条不需要增加点数，需要将首尾点替换掉
	for(int i = 0; i < Num; i++)
		temp[i] = pt[i];
 
	temp[0].x_node = 2*temp[0].x_node - temp[1].x_node; // 将折线两端点换成延长线上两点
	temp[0].y_node = 2*temp[0].y_node - temp[1].y_node;
 
	temp[Num-1].x_node = 2*temp[Num-1].x_node - temp[Num-2].x_node;
	temp[Num-1].y_node = 2*temp[Num-1].y_node - temp[Num-2].y_node;
 
	//delete []pt;  // 点数由原来的Num个增加到Num+InsertNumSum个，删除旧的存储空间，开辟新的存储空间
    pt.clear();
	//pt = new Node[Num + InsertNumSum];
    pt.resize(Num + InsertNumSum);           
 
	Node NodePt1, NodePt2, NodePt3, NodePt4;// 两节点间均匀插入点，需要相邻两段样条曲线,因此需要四个节点
 
	double t;
	int totalnum = 0;
	for(int i = 0; i < Num - 1; i++) // 每条线段均匀插入点
	{
		if(i == 0){ // 第一段只需计算第一条样条曲线，无NodePt1   
			NodePt2 = temp[i]; 
            NodePt3 = temp[i+1]; 
            NodePt4 = temp[i+2];     
 
			double dt = 0.5/(InsertNum[i] + 1);
			for(int j = 0; j < InsertNum[i] + 1; j++)
			{
				t = 0 + dt*j;
				pt[totalnum].x_node = F02(t)*NodePt2.x_node + F12(t)*NodePt3.x_node + F22(t)*NodePt4.x_node;
				pt[totalnum].y_node = F02(t)*NodePt2.y_node + F12(t)*NodePt3.y_node + F22(t)*NodePt4.y_node;
				totalnum++;
			}
		}else if(i == Num - 2){ // 最后一段只需计算最后一条样条曲线，无NodePt4
			NodePt1 = temp[i-1]; 
            NodePt2 = temp[i]; 
            NodePt3 = temp[i+1];
 
			double dt = 0.5/(InsertNum[i] + 1);
			for(int j = 0; j < InsertNum[i] + 2; j++)
			{
				t = 0.5 + dt*j;
				pt[totalnum].x_node = F02(t)*NodePt1.x_node + F12(t)*NodePt2.x_node + F22(t)*NodePt3.x_node;
				pt[totalnum].y_node = F02(t)*NodePt1.y_node + F12(t)*NodePt2.y_node + F22(t)*NodePt3.y_node;
				totalnum++;
			}
		}else{
			NodePt1 = temp[i-1];
            NodePt2 = temp[i]; 
            NodePt3 = temp[i+1]; 
            NodePt4 = temp[i+2]; //NodePt1,2,3计算第一条曲线，NodePt2,3,4计算第二条曲线
 
			int LeftInsertNum, RightInsertNum;// 计算线段间左右曲线段上分别需要插入的点数
			double rightoffset = 0; // 左边曲线段从t=0.5开始，又边曲线段从t=rightoffset开始
			double Leftdt = 0, Rightdt = 0; // 左右曲线取点t步长
			if(InsertNum[i] == 0){
				LeftInsertNum = 0;
				RightInsertNum = 0;
			}else if(InsertNum[i] % 2 == 1){// 插入点数为奇数，左边曲线段插入点个数比右边多一点
				RightInsertNum = InsertNum[i]/2;
				LeftInsertNum = RightInsertNum + 1;
				Leftdt = 0.5/(LeftInsertNum);
				Rightdt = 0.5/(RightInsertNum + 1);
				rightoffset = Rightdt;
			}else{ // 插入点数为偶数，左右边曲线段插入个数相同
				RightInsertNum = InsertNum[i]/2;
				LeftInsertNum = RightInsertNum;
				Leftdt = 0.5/(LeftInsertNum + 0.5);
				Rightdt = 0.5/(RightInsertNum + 0.5);
				rightoffset = Rightdt/2;
			}
 
			for(int j = 0; j < LeftInsertNum + 1; j++)
			{
				t = 0.5 + Leftdt*j;
				pt[totalnum].x_node = F02(t)*NodePt1.x_node + F12(t)*NodePt2.x_node + F22(t)*NodePt3.x_node;
				pt[totalnum].y_node = F02(t)*NodePt1.y_node + F12(t)*NodePt2.y_node + F22(t)*NodePt3.y_node;
				totalnum++;
			}
 
			for(int j = 0; j < RightInsertNum; j++)
			{				
				t = rightoffset + Rightdt*j;
				pt[totalnum].x_node = F02(t)*NodePt2.x_node + F12(t)*NodePt3.x_node + F22(t)*NodePt4.x_node;
				pt[totalnum].y_node = F02(t)*NodePt2.y_node + F12(t)*NodePt3.y_node + F22(t)*NodePt4.y_node;
				totalnum++;
			}
		}
	}
	delete []temp;
	Num = Num + InsertNumSum;
 
}
//================================================================
// 函数功能： 二次样条基函数
//
//================================================================
double CBSpline::F02(double t)
{
	return 0.5*(t - 1)*(t - 1);
}
double CBSpline::F12(double t)
{
	return 0.5*(-2*t*t + 2*t + 1);
}
double CBSpline::F22(double t)
{
	return 0.5*t*t;
}
//========================================================================
// 函数功能： 三次B样条平滑，把给定的点，平滑到B样条曲线上，不增加点的数目
// 输入参数： *pt ：给定点序列，执行完成后，会被替换成新的平滑点
//            Num：点个数
// 返回值：   无返回值
//
//========================================================================
void CBSpline::ThreeOrderBSplineSmooth(std::vector<Node> &pt, int Num)
{
	Node *temp = new Node[Num + 2];
	for(int i = 0; i < Num; i++)
		temp[i+1] = pt[i];
 
	temp[0].x_node = 2*temp[1].x_node - temp[2].x_node; // 将折线延长线上两点加入作为首点和尾点
	temp[0].y_node = 2*temp[1].y_node - temp[2].y_node;
 
	temp[Num+1].x_node = 2*temp[Num].x_node - temp[Num-1].x_node;
	temp[Num+1].y_node = 2*temp[Num].y_node - temp[Num-1].y_node;
 
	Node NodePt1, NodePt2, NodePt3, NodePt4;
	double t;
	for(int i = 0; i < Num - 1; i++)
	{
		NodePt1 = temp[i]; 
        NodePt2 = temp[i+1]; 
        NodePt3 = temp[i+2]; 
        NodePt4 = temp[i+3];
 
		if(i == Num - 4){ // 最后一段取t=0.5和t=1点
			t = 0;
			pt[i].x_node = F03(t)*NodePt1.x_node + F13(t)*NodePt2.x_node + F23(t)*NodePt3.x_node + F33(t)*NodePt4.x_node;
			pt[i].y_node = F03(t)*NodePt1.y_node + F13(t)*NodePt2.y_node + F23(t)*NodePt3.y_node + F33(t)*NodePt4.y_node;
			t = 1;
			pt[i+1].x_node = F03(t)*NodePt1.x_node + F13(t)*NodePt2.x_node + F23(t)*NodePt3.x_node + F33(t)*NodePt4.x_node;
			pt[i+1].y_node = F03(t)*NodePt1.y_node + F13(t)*NodePt2.y_node + F23(t)*NodePt3.y_node + F33(t)*NodePt4.y_node;
		}else{ // 中间段取t=0.5点
			t = 0;
			pt[i].x_node = F03(t)*NodePt1.x_node + F13(t)*NodePt2.x_node + F23(t)*NodePt3.x_node + F33(t)*NodePt4.x_node;
			pt[i].y_node = F03(t)*NodePt1.y_node + F13(t)*NodePt2.y_node + F23(t)*NodePt3.y_node + F33(t)*NodePt4.y_node;
		}
	}
	delete []temp;
}
 
//================================================================
// 函数功能： 三次B样条拟合,在节点之间均匀插入指定个数点
// 输入参数： *pt ：给定点序列，执行完成后，会被替换成新的数据点
//            Num：节点点个数
//            *InsertNum: 节点之间需要插入的点个数指针 
// 返回值：   无返回值
//
//=================================================================
void CBSpline::ThreeOrderBSplineInterpolatePt(std::vector<Node> &pt, int Num, int *InsertNum)
{
	if(pt.size() < 2 || InsertNum == NULL) return;
 
	int InsertNumSum = 0;// 计算需要插入的点总数
	for(int i = 0; i < Num - 1; i++)  InsertNumSum += InsertNum[i];
 
	Node *temp = new Node[Num + 2];
	for(int i = 0; i < Num; i++)
		temp[i + 1] = pt[i];
 
	temp[0].x_node = 2*temp[1].x_node - temp[2].x_node;// 将折线延长线上两点加入作为首点和尾点
	temp[0].y_node = 2*temp[1].y_node - temp[2].y_node;
 
	temp[Num+1].x_node = 2*temp[Num].x_node - temp[Num-1].x_node;
	temp[Num+1].y_node = 2*temp[Num].y_node - temp[Num-1].y_node;
 
	Node NodePt1, NodePt2, NodePt3, NodePt4;
	double t;
 
	//delete []pt;// 点数由原来的Num个增加到Num+InsertNumSum个，删除旧的存储空间，开辟新的存储空间
    pt.clear();
	//pt = new Node[Num + InsertNumSum];
    pt.resize(Num + InsertNumSum);           
 
	int totalnum = 0;
	for(int i = 0; i < Num - 1; i++) // 每条线段均匀插入点
	{
		NodePt1 = temp[i]; 
        NodePt2 = temp[i+1]; 
        NodePt3 = temp[i+2]; 
        NodePt4 = temp[i+3];
		double dt = 1.0/(InsertNum[i] + 1);
 
		for(int j = 0; j < InsertNum[i] + 1; j++)
		{
			t = dt*j;
			pt[totalnum].x_node = F03(t)*NodePt1.x_node + F13(t)*NodePt2.x_node + F23(t)*NodePt3.x_node + F33(t)*NodePt4.x_node;
			pt[totalnum].y_node = F03(t)*NodePt1.y_node + F13(t)*NodePt2.y_node + F23(t)*NodePt3.y_node + F33(t)*NodePt4.y_node;
			totalnum++;
		}
 
		if(i == Num - 2){ // 最后一个尾点
			t = 1;
			pt[totalnum].x_node = F03(t)*NodePt1.x_node + F13(t)*NodePt2.x_node + F23(t)*NodePt3.x_node + F33(t)*NodePt4.x_node;
			pt[totalnum].y_node = F03(t)*NodePt1.y_node + F13(t)*NodePt2.y_node + F23(t)*NodePt3.y_node + F33(t)*NodePt4.y_node;
			totalnum++;
		}
	}
 
	delete []temp;
	Num = Num + InsertNumSum;
 
}

//================================================================
// 函数功能： 三次样条基函数
//
//================================================================
double CBSpline::F03(double t)
{
	return 1.0/6*(-t*t*t + 3*t*t - 3*t + 1);
}
double CBSpline::F13(double t)
{
	return 1.0/6*(3*t*t*t - 6*t*t + 4);
}
double CBSpline::F23(double t)
{
	return 1.0/6*(-3*t*t*t + 3*t*t + 3*t + 1);
}
double CBSpline::F33(double t)
{
	return 1.0/6*t*t*t;
}

void GlobalPlan::run()
{
    /*createmap();
    //验证起点、终点知否超过索引
    if(start_position_ > maxnodenumber || end_position_ > maxnodenumber)
    {
        ROS_ERROR("out of start or end position index");
        ros::spin();
    }
    //验证起点、终点是否存在
    if(map[start_position_]->data == Node(-1) || map[end_position_]->data == Node(-1))//findnode(start_position_)
    {
        ROS_ERROR("illegal start or end position");
        ros::spin();
    }*/
    //DFS(start_position_, end_position_);
    //Astar(start_position_, end_position_);
    //Dijkstra(start_position_, end_position_);
    /*if(isfindpath)
    {
        for(int i = 0; i < pathres.size(); i++)
        {
            if(i != pathres.size()-1)
            {
                //std::cout << pathres[i] << " " << Astarmap[pathres[i]].F << " ---> " ;
                std::cout << pathres[i] << " ---> " ;
            }else{
                std::cout << pathres[i];
            }
        }
        std::cout << std::endl;
        std::cout << "cost: " << Astarmap[*(--pathres.end())].F << std::endl;
        findsmoothpath();
    }
    
    pathvisual();
    ros::Rate r(10);
    while(n.ok())
    {
        map_pub.publish(map_marker);
        path_pub.publish(path_marker);
        r.sleep();
        ros::spinOnce();
    }*/

    //
    while(start_position_ != end_position_ && n.ok())
    {
        createmap();
        //验证起点、终点知否超过索引
        if(start_position_ > maxnodenumber || end_position_ > maxnodenumber)
        {
            ROS_ERROR("out of start or end position index");
            ros::spin();
        }
        //验证起点、终点是否存在
        if(map[start_position_]->data == Node(-1) || map[end_position_]->data == Node(-1))//findnode(start_position_)
        {
            ROS_ERROR("illegal start or end position");
            ros::spin();
        }
        Astar(start_position_, end_position_);
        if(isfindpath)
        {
            for(int i = 0; i < pathres.size(); i++)
            {
                if(i != pathres.size()-1)
                {
                    //std::cout << pathres[i] << " " << Astarmap[pathres[i]].F << " ---> " ;
                    std::cout << pathres[i] << " ---> " ;
                }else{
                    std::cout << pathres[i];
                }
            }
            std::cout << std::endl;
            std::cout << "cost: " << Astarmap[*(--pathres.end())].F << std::endl;
            findsmoothpath();
        }
        ros::Rate r(0.5);
        pathvisual();
        map_pub.publish(map_marker);
        path_pub.publish(path_marker);
        r.sleep();
        start_position_ = pathres[1];
        //std::cout << pathres[1] << std::endl;
        for(int i = 0; i <= std::max((int)(nodewithid.size()), maxnodenumber); ++i)
        {
            delete []map[i];
        }
        delete []map;
        pathres.clear();
        isfindpath = false;
    }
    ROS_INFO("finished !!!");
    ros::spin();
}


