/*
 * @Author: your name
 * @Date: 2021-11-23 16:07:05
 * @LastEditTime: 2021-11-29 15:07:24
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/src/AStar_ros.cpp
 */
#include "plan/AStar_ros.h"


AStar_ros::AStar_ros()
{
}

AStar_ros::AStar_ros(Node* start, Node* end, const std::vector<std::vector<int>> &map_, const bool isIgnoreCorner_):
    AStar(start, end, map_, isIgnoreCorner_)
{
    pathpub = n.advertise<nav_msgs::Path>("/path", 1);
    mappub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    pointpub = n.advertise<geometry_msgs::PoseArray>("/point", 1);
    flag = false;
    //std::cout << "child born" << std::endl;
}

AStar_ros::~AStar_ros()
{
}

void AStar_ros::visual()
{
    nav_msgs::OccupancyGrid gridmap;
    currenttime = ros::Time::now();
    gridmap.header.frame_id = "map";
    gridmap.header.stamp = currenttime; 
    gridmap.info.resolution = 1;         // float32
    gridmap.info.width      = map.size();           // uint32
    gridmap.info.height     = map[0].size();           // uint32
    int mapwidth = map.size();
    gridmap.data.resize(gridmap.info.height * gridmap.info.width);
    for(int i = 0; i < gridmap.data.size(); ++i)
    {
        gridmap.data[i] = map[i%mapwidth][i/mapwidth] * 100;//数据列排列
        for(auto iter = closelist.begin(); iter != closelist.end(); ++iter)
        {
            if(i == (*iter)->x + (*iter)->y * mapwidth)
            {
                gridmap.data[i] = 30;
            }
        }
    }
    mappub.publish(gridmap);

    geometry_msgs::Pose startpoint, endpoint;
    startpoint.position.x = start_node->x + 0.5;
    startpoint.position.y = start_node->y + 0.5;
    startpoint.position.z = 0;
    startpoint.orientation.x = 0;
    startpoint.orientation.y = 0;
    startpoint.orientation.z = 0;
    startpoint.orientation.w = 1;
    endpoint.position.x = end_node->x + 0.5;
    endpoint.position.y = end_node->y + 0.5;
    endpoint.position.z = 0;
    endpoint.orientation = startpoint.orientation;
    geometry_msgs::PoseArray start_end;
    start_end.header.frame_id = "map";
    start_end.header.stamp = currenttime;
    start_end.poses.push_back(startpoint);
    start_end.poses.push_back(endpoint);
    pointpub.publish(start_end);
    ros::Rate loop_rate(5);
    loop_rate.sleep();

    //tf
    if(!flag)
    {
        tf::Quaternion q;
        q.setRPY(0, 0, -1.57);
        baselink2map.setRotation(q);
        baselink2map.setOrigin(tf::Vector3(start_node->x + 0.5, start_node->y + 0.5, 0.055));
        broadcaster.sendTransform(tf::StampedTransform(baselink2map, currenttime, "map", "base_link"));
    }
}

void AStar_ros::visual(std::list<Node*> visuallist)
{
    visual();
    if(visuallist.empty()) return;
    nav_msgs::Path path;
    path.header.stamp = currenttime;
    path.header.frame_id = "map";
    for(auto iter = visuallist.begin(); iter != visuallist.end(); ++iter)
    {
        geometry_msgs::PoseStamped point;
        point.pose.position.x = (*iter)->x + 0.5;
        point.pose.position.y = (*iter)->y + 0.5;
        point.pose.position.z = 0;
        point.pose.orientation.x = 0;
        point.pose.orientation.y = 0;
        point.pose.orientation.z = 0;
        point.pose.orientation.w = 1;
        point.header.stamp = currenttime;
        point.header.frame_id = "map";
        path.poses.push_back(point);
    }
    pathpub.publish(path);

    //tf
    if(flag)
    {
        for(auto iter = visuallist.begin(); iter != visuallist.end(); ++iter)
        {
            tf::Quaternion q;
            double distance = 0;
            if((*iter)->father != nullptr)
            {
                distance = std::sqrt(((*iter)->x - (*iter)->father->x) * ((*iter)->x - (*iter)->father->x)
                            + ((*iter)->y - (*iter)->father->y) * ((*iter)->y - (*iter)->father->y));
                if(((*iter)->y != (*iter)->father->y))
                {
                    double temp = ((*iter)->x - (*iter)->father->x) / ((*iter)->y - (*iter)->father->y);
                    q.setRPY(0, 0, -temp);
                }else{
                    //x,y不同时为0
                    double temp = std::fabs((*iter)->x - (*iter)->father->x) / ((*iter)->x - (*iter)->father->x);
                    q.setRPY(0, 0, -temp*1.57);
                }
            }else{
                q.setRPY(0, 0, -1.57);
            }
            if(distance == 0)
            {
                baselink2map.setRotation(q);
                baselink2map.setOrigin(tf::Vector3((*iter)->x + 0.5 , (*iter)->y + 0.5 , 0.055));
                broadcaster.sendTransform(tf::StampedTransform(baselink2map, currenttime, "map", "base_link"));
                ros::Rate loop_rate(5);
                loop_rate.sleep();
                continue;
            }
            for(int i = 0; i < (int)distance/0.1; ++i)
            {
                if(distance != 1)
                {
                    baselink2map.setRotation(q);
                    int symbolx = ((*iter)->x - (*iter)->father->x) / std::fabs((*iter)->x - (*iter)->father->x);
                    int symboly = ((*iter)->y - (*iter)->father->y) / std::fabs((*iter)->y - (*iter)->father->y);
                    baselink2map.setOrigin(tf::Vector3((*iter)->father->x + 0.5 + symbolx*i*0.1, (*iter)->father->y + 0.5 + symboly*i*0.1, 0.055));
                    broadcaster.sendTransform(tf::StampedTransform(baselink2map, currenttime, "map", "base_link"));
                    ros::Rate loop_rate(5);
                    loop_rate.sleep();
                }else{
                    if((*iter)->x == (*iter)->father->x)
                    {
                        baselink2map.setRotation(q);
                        int symboly = ((*iter)->y - (*iter)->father->y) / std::fabs((*iter)->y - (*iter)->father->y);
                        baselink2map.setOrigin(tf::Vector3((*iter)->father->x + 0.5, (*iter)->father->y + 0.5 + symboly*i*0.1, 0.055));
                        broadcaster.sendTransform(tf::StampedTransform(baselink2map, currenttime, "map", "base_link"));
                        ros::Rate loop_rate(5);
                        loop_rate.sleep();
                    }else{
                        baselink2map.setRotation(q);
                        int symbolx = ((*iter)->x - (*iter)->father->x) / std::fabs((*iter)->x - (*iter)->father->x);
                        baselink2map.setOrigin(tf::Vector3((*iter)->father->x + 0.5 + symbolx*i*0.1, (*iter)->father->y + 0.5, 0.055));
                        broadcaster.sendTransform(tf::StampedTransform(baselink2map, currenttime, "map", "base_link"));
                        ros::Rate loop_rate(5);
                        loop_rate.sleep();
                    }
                }
            }
        }
        tf::Quaternion q;
        q.setRPY(0, 0, -1.57);
        baselink2map.setRotation(q);
        baselink2map.setOrigin(tf::Vector3(end_node->x + 0.5, end_node->y + 0.5, 0.055));
        broadcaster.sendTransform(tf::StampedTransform(baselink2map, currenttime, "map", "base_link"));
    }
}

bool AStar_ros::search()
{
    openlist.push_back(start_node);//输入起点
    while(!openlist.empty())
    {
        Node* curnode= getLeastFnode();//找到F值最小的点
        
        openlist.remove(curnode);//从开启列表中删除
        closelist.push_back(curnode); //放到关闭列表
        //1,找到当前周围八个格中可以通过的格子
        std::vector<Node*> condidates = getNextNode(curnode);
        //std::cout << "child search" <<std::endl;
        for(int i = 0; i < condidates.size(); ++i)
        {
            //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
            if(!isInOpenlist(condidates[i]))
            {
                condidates[i]->father = curnode;
                condidates[i]->g = calcG(curnode, condidates[i]);
                condidates[i]->h = calcH(condidates[i]);
                condidates[i]->f = calcF(condidates[i]);
                openlist.push_back(condidates[i]);
            }else{
                //3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
                int tempg = calcG(curnode, condidates[i]);
                if(tempg > condidates[i]->g)
                {
                    continue;
                }else{
                    condidates[i]->father = curnode;
                    condidates[i]->g = tempg;
                    condidates[i]->f = calcF(condidates[i]);
                }
                delete condidates[i];
                condidates[i] = nullptr;
            }
            //如果结束点出现在openlist则搜索成功
            if(isInOpenlist(end_node)) 
            {
                end_node->father = curnode;
                return true;
            }
        }
        visual();
    }
    return false;
}

std::list<Node*> AStar_ros::solve()
{
    std::list<Node*> path;
    Node* pathnode = end_node;
    if(search())
    {
        while(pathnode != nullptr)
        {
            path.push_front(pathnode);
            pathnode = pathnode->father;
            //std::cout << "child solve" <<std::endl;
        }
        flag = true;
    }
    visual(path);
    openlist.clear();
    closelist.clear();

    return path;
}






