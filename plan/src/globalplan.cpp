/*
 * @Author: your name
 * @Date: 2021-11-28 19:27:18
 * @LastEditTime: 2022-04-05 14:46:20
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/src/globalplan.cpp
 */
#include "plan/globalplan.h"

GlobalPlan::GlobalPlan(ros::NodeHandle n_)
{
    n = n_;
    flag_start = false;
    flag_end = false;
    flag_map = false;
    flag_search = false;
    flag_finish = false;
    currenttime = ros::Time::now();
    pub_path = n.advertise<nav_msgs::Path>("/path", 1);
    pub_startpoint = n.advertise<geometry_msgs::PoseStamped>("/start_point", 1);
    pub_endpoint = n.advertise<geometry_msgs::PoseStamped>("/end_point", 1);
    pub_map = n.advertise<nav_msgs::OccupancyGrid>("/new_map", 1);
    sub_point = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &GlobalPlan::global_point_callback, this);
    sub_map = n.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &GlobalPlan::global_map_callback, this);
    sub_end = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GlobalPlan::global_end_callback, this);
    sub_start = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, &GlobalPlan::global_start_callback, this);
    
    //boost::thread thtf(boost::bind(&GlobalPlan::tf_update, this));
    //thtf.detach();
    /*while(n.ok())
    {
        sub_start = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, &GlobalPlan::global_start_callback, this);
        sub_end = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GlobalPlan::global_end_callback, this);
        sub_map = n.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &GlobalPlan::global_map_callback, this);
        if(!flag_start || !flag_end || !flag_map)
        {
            tf_publisher(0.0, tf::Vector3(0, 0, 0.055));
        }else{
            if(!flag_search)
            {
                double yaw_, pitch_, roll_;
                tf::Quaternion qtemp(global_start_orientation.x, global_start_orientation.y, global_start_orientation.z, global_start_orientation.w);
                tf::Matrix3x3(qtemp).getEulerYPR(yaw_, pitch_, roll_);
                std::cout << yaw_ << " " << pitch_ << " " << roll_ << std::endl;
                tf_publisher(yaw_-pi/2, tf::Vector3(global_start->x + 0.5 , global_start->y + 0.5 , 0.055));
            }
        }
        ros::spinOnce();
        ros::Rate main_sleep(10);
        main_sleep.sleep();
    }*/
    //ros::AsyncSpinner spinner(3);
    //spinner.start();
    //ros::waitForShutdown();
    ros::spin();
}

GlobalPlan::~GlobalPlan()
{
    //delete astar;
    delete global_start;
    global_start = nullptr;
    delete global_end;
    global_end = nullptr;
}

void GlobalPlan::global_point_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    int x = msg->point.x;
    int y = msg->point.y;
    ROS_INFO("point x=%d, y=%d", x, y);
    if(y < global_map.size() && x < global_map[0].size())
    {
        global_map[x][y] = 1;
        newmap.data[y*newmap.info.width+x] = 100;
        pub_map.publish(newmap);
    }else{
        ROS_INFO("out of map!");
    }
}

void GlobalPlan::global_start_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    //delete global_start;
    //global_start = nullptr;
    int x = msg->pose.pose.position.x;
    int y = msg->pose.pose.position.y;
    global_start_orientation = msg->pose.pose.orientation;
    ROS_INFO("start x=%d, y=%d", x, y);
    global_start = new Node(x, y);
    flag_start = true;
    ROS_INFO("start get");
    geometry_msgs::PoseStamped startpoint;
    startpoint.pose.position.x = global_start->x + 0.5;
    startpoint.pose.position.y = global_start->y + 0.5;
    startpoint.pose.position.z = 0;
    startpoint.pose.orientation = global_start_orientation;
    startpoint.header.frame_id = "map";
    startpoint.header.stamp = currenttime;
    pub_startpoint.publish(startpoint);
    // if(!flag_search)
    // {
    //     double yaw_, pitch_, roll_;
    //     tf::Quaternion qtemp(global_start_orientation.x, global_start_orientation.y, global_start_orientation.z, global_start_orientation.w);
    //     tf::Matrix3x3(qtemp).getEulerYPR(yaw_, pitch_, roll_);
    //     std::cout << yaw_ << " " << pitch_ << " " << roll_ << std::endl;
    //     tf_publisher(yaw_-pi/2, tf::Vector3(global_start->x + 0.5 , global_start->y + 0.5 , 0.055));
    // }
}

void GlobalPlan::global_end_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //delete global_end;
    //global_end = nullptr;
    int x = msg->pose.position.x;
    int y = msg->pose.position.y;
    global_end_orientation = msg->pose.orientation;
    ROS_INFO("end x=%d, y=%d", x, y);
    global_end = new Node(x, y);
    flag_end = true;
    ROS_INFO("end get");
    geometry_msgs::PoseStamped endpoint;
    endpoint.pose.position.x = global_end->x + 0.5;
    endpoint.pose.position.y = global_end->y + 0.5;
    endpoint.pose.position.z = 0;
    endpoint.pose.orientation = global_end_orientation;
    endpoint.header.frame_id = "map";
    endpoint.header.stamp = currenttime;
    pub_startpoint.publish(endpoint);
    visual(global_solve());
    flag_finish = true;
}

void GlobalPlan::global_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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

    newmap.header = msg->header;
    newmap.info = msg->info;
    newmap.data = msg->data;
    pub_map.publish(newmap);
    ROS_INFO("map get");
    //tf_publisher(0, tf::Vector3(0, 0, 0.055));
    //global_start = new Node(1, 1);
    //global_end = new Node(5, 6);
    //global_solve();
}

std::list<Node*> GlobalPlan::global_solve()
{
    std::list<Node*> global_result;
    ROS_INFO("searching");
    //std::cout << "start in " << global_start->x << " " << global_start->y << std::endl;
    astar = new AStar(global_start, global_end, global_map);
    global_result = astar->solve();
    flag_search = true;

    // dstar = new DStar(global_map);
    // dstar->DstarRun();
    // dstar->printPath();
    return global_result;
}

void GlobalPlan::visual(const std::list<Node*> visual_path)
{
    //if(visual_path.empty()) return;
    //visual path
    nav_msgs::Path path;
    path.header.stamp = currenttime;
    path.header.frame_id = "map";
    for(auto iter = visual_path.begin(); iter != visual_path.end(); ++iter)
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
    pub_path.publish(path);

    //
    // double yaw2d;
    // for(auto iter = visual_path.begin(); iter != visual_path.end(); ++iter)
    // {
    //     double distance = 0;
    //     if((*iter)->father != nullptr)
    //     {
    //         distance = std::sqrt(((*iter)->x - (*iter)->father->x) * ((*iter)->x - (*iter)->father->x)
    //                     + ((*iter)->y - (*iter)->father->y) * ((*iter)->y - (*iter)->father->y));
    //         if(((*iter)->x != (*iter)->father->x))
    //         {
    //             yaw2d = -pi/2 + std::atan2(((*iter)->y - (*iter)->father->y) , ((*iter)->x - (*iter)->father->x));
    //             //std::cout << yaw2d << std::endl;
    //         }else{
    //             //x,y不同时为0
    //             if((*iter)->y < (*iter)->father->y)
    //             {
    //                 yaw2d = pi;
    //             }else{
    //                 yaw2d = 0;
    //             }
    //             //std::cout << yaw2d << std::endl;
    //         }
    //     }
    //     if(distance == 0){
    //         double yaw_, pitch_, roll_;
    //         tf::Quaternion qtemp(global_start_orientation.x, global_start_orientation.y, global_start_orientation.z, global_start_orientation.w);
    //         tf::Matrix3x3(qtemp).getEulerYPR(yaw_, pitch_, roll_);
    //         tf_publisher(yaw_-pi/2, tf::Vector3((*iter)->x + 0.5 , (*iter)->y + 0.5 , 0.055));
    //         continue;
    //     }
    //     for(int i = 0; i < (int)distance/0.1; ++i)
    //     {
    //         if(distance != 1)
    //         {
    //             int symbolx = ((*iter)->x - (*iter)->father->x) / std::fabs((*iter)->x - (*iter)->father->x);
    //             int symboly = ((*iter)->y - (*iter)->father->y) / std::fabs((*iter)->y - (*iter)->father->y);
    //             tf_publisher(yaw2d, tf::Vector3((*iter)->father->x + 0.5 + symbolx*i*0.1, (*iter)->father->y + 0.5 + symboly*i*0.1, 0.055));
    //         }else{
    //             if((*iter)->x == (*iter)->father->x)
    //             {
    //                 int symboly = ((*iter)->y - (*iter)->father->y) / std::fabs((*iter)->y - (*iter)->father->y);
    //                 tf_publisher(yaw2d, tf::Vector3((*iter)->father->x + 0.5, (*iter)->father->y + 0.5 + symboly*i*0.1, 0.055));
    //             }else{
    //                 int symbolx = ((*iter)->x - (*iter)->father->x) / std::fabs((*iter)->x - (*iter)->father->x);
    //                 tf_publisher(yaw2d, tf::Vector3((*iter)->father->x + 0.5 + symbolx*i*0.1, (*iter)->father->y + 0.5, 0.055));
    //             }
    //         }
    //     }
    // }
    // double yaw_, pitch_, roll_;
    // tf::Quaternion qtemp(global_end_orientation.x, global_end_orientation.y, global_end_orientation.z, global_end_orientation.w);
    // tf::Matrix3x3(qtemp).getEulerYPR(yaw_, pitch_, roll_);
    // tf_publisher(yaw_-pi/2, tf::Vector3(global_end->x + 0.5, global_end->y + 0.5, 0.055));
}

void GlobalPlan::tf_publisher(const double yaw2d_, const tf::Vector3 tf_move)
{
    //if(flag_search) return;
    tf::Quaternion q;
    q.setRPY(0, 0, yaw2d_);
    baselink2map.setRotation(q);
    baselink2map.setOrigin(tf_move);
    broadcaster.sendTransform(tf::StampedTransform(baselink2map, currenttime, "map", "base_link"));
    ros::Rate tf_sleep(5);
    tf_sleep.sleep();
}


void GlobalPlan::tf_update()
{
    if(!flag_search)
    {
        tf_publisher(0, tf::Vector3(0.5, 0.5, 0.055));
    }
    if(flag_finish)
    {
        double yaw_, pitch_, roll_;
        tf::Quaternion qtemp(global_end_orientation.x, global_end_orientation.y, global_end_orientation.z, global_end_orientation.w);
        tf::Matrix3x3(qtemp).getEulerYPR(yaw_, pitch_, roll_);
        tf_publisher(yaw_-pi/2, tf::Vector3(global_end->x + 0.5, global_end->y + 0.5, 0.055));
    }
}


