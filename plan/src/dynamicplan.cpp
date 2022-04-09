/*
 * @Author: your name
 * @Date: 2022-04-05 13:53:17
 * @LastEditTime: 2022-04-07 16:55:43
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/plan/src/dynamicplan.cpp
 */
#include "../include/plan/dynamicplan.h"

DynamicPlan::DynamicPlan(ros::NodeHandle n_):nh(n_)
{
    flag_start = false;
    flag_end = false;
    flag_map = false;
    flag_newmap = false;
    path_pub = nh.advertise<nav_msgs::Path>("/path", 1);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/new_map", 1);
    ros::Rate r(10);
    while(nh.ok())
    {
        currenttime = ros::Time::now();
        point_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &DynamicPlan::point_callback, this);
        map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &DynamicPlan::map_callback, this);
        goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &DynamicPlan::goal_callback, this);
        start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, &DynamicPlan::start_callback, this);
        r.sleep();
        ros::spinOnce();
        //ros::spin();
    }
}

void DynamicPlan::point_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    int x = msg->point.x;
    int y = msg->point.y;
    ROS_INFO("point x=%d, y=%d", x, y);
    if(y < global_map.size() && x < global_map[0].size())
    {
        global_map[x][y] = 1;
        newmap.data[y*newmap.info.width+x] = 100;
        map_pub.publish(newmap);

        //D*
        dstar->changeMap(x, y);
        dstar->RePlan();
        std::vector<DNode> findpaths = dstar->findPath();
        //std::cout << "final node next x= " << findpaths.end()->nextx << ", y= " << findpaths.end()->nexty << std::endl;
        std::cout << "final node next x= " << findpaths[findpaths.size()-1].nextx << ", y= " << findpaths[findpaths.size()-1].nexty << std::endl;
        visual(findpaths);
        //std::cout << "D* finish" << std::endl;
    }else{
        ROS_INFO("out of map!");
    }
}

void DynamicPlan::start_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    int x = msg->pose.pose.position.x;
    int y = msg->pose.pose.position.y;
    //global_start_orientation = msg->pose.pose.orientation;
    ROS_INFO("start x=%d, y=%d", x, y);
    start_node = DNode(x, y);
    flag_start = true;
    ROS_INFO("start get");
}

void DynamicPlan::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    int x = msg->pose.position.x;
    int y = msg->pose.position.y;
    //global_goal_orientation = msg->pose.orientation;
    ROS_INFO("end x=%d, y=%d", x, y);
    goal_node = DNode(x, y);
    flag_end = true;
    ROS_INFO("end get");

    //plan
    // DStar dstar(global_map);
    dstar->DstarRun(start_node, goal_node);
    dstar->isCanReachGoal(start_node);
    std::vector<DNode> findpaths = dstar->findPath();
    // for(int i = 0; i < findpaths.size(); ++i)
    // {
    //     std::cout << "x= " << findpaths[i].x << ", y= " << findpaths[i].y << " : " << findpaths[i].K << std::endl;
    // }
    //std::cout << "final node next x= " << findpaths[findpaths.size()-1].nextx << ", y= " << findpaths[findpaths.size()-1].nexty << std::endl;
    visual(findpaths);
}

void DynamicPlan::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    int width_ = msg->info.height;
    int height_ = msg->info.width;
    for(int i = 0; i < height_; ++i)
    {
        std::vector<int> map_line;
        for(int j = 0; j < width_; ++j)
        {
            map_line.push_back(msg->data[i + j * height_]/100);
        }
        global_map.push_back(map_line);
    }
    flag_map = true;

    if(!flag_newmap)
    {
        dstar = new DStar(global_map);
        newmap.header = msg->header;
        newmap.info = msg->info;
        newmap.data = msg->data;
        // map_pub.publish(newmap);
        ROS_INFO("map get");
        flag_newmap = true;
    }
    map_pub.publish(newmap);
    
}

void DynamicPlan::visual(std::vector<DNode> &path_)
{
    nav_msgs::Path path;
    path.header.stamp = currenttime;
    path.header.frame_id = "map";
    for(int i = 0; i < path_.size(); ++i)
    {
        geometry_msgs::PoseStamped point;
        point.pose.position.x = path_[i].x + 0.5;
        point.pose.position.y = path_[i].y + 0.5;
        point.pose.position.z = 0;
        point.pose.orientation.x = 0;
        point.pose.orientation.y = 0;
        point.pose.orientation.z = 0;
        point.pose.orientation.w = 1;
        point.header.stamp = currenttime;
        point.header.frame_id = "map";
        path.poses.push_back(point);
    }
    path_pub.publish(path);
}

DynamicPlan::~DynamicPlan()
{
    delete dstar;
}