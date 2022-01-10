/*
 * @Author: your name
 * @Date: 2021-12-28 11:22:46
 * @LastEditTime: 2022-01-10 16:37:25
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/include/graph_tool/tool_core.h
 */
#ifndef TOOL_CORE_H_
#define TOOL_CORE_H_

#include "tinyxml.h"
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <std_msgs/Int8.h>
//#include "graph_tool/SetPath.h"
//#include "graph_tool/PathPoint.h"
#include <dynamic_reconfigure/server.h>
#include "graph_tool/modeConfig.h"

struct Node
{
    int id_node;
    double x_node;
    double y_node;
};

struct Path
{
    int source_path;
    int target_path;
    double distance_path;
    double angel_path;
};

class GraphTool
{
private:
    ros::NodeHandle n;
    ros::Subscriber point_sub;
    ros::Publisher point_pub;
    ros::Publisher path_pub;
    visualization_msgs::MarkerArray points;
    visualization_msgs::MarkerArray paths;
    std::vector<Node> nodewithid;
    std::vector<Path> pathwithpoints;
    std::vector<std::string> key_name;
    std::vector<std::string> key_type;
    std::vector<std::string> key_for;
    std::vector<std::string> key_id;
    std::string map_path_;
    std::string map_name_;
    std::string map_path_save;
    double searchradius;
    TiXmlDocument doc;
    TiXmlElement *head_write;
    bool isnew_;
    int count;
    int pathnumber;
    bool iscatch, isaddpath, isdeletepoint, isdeletepath;
    bool two_node;
    Node pathnodea, pathnodeb;
    dynamic_reconfigure::Server<set_mode::modeConfig> mode_server;
    dynamic_reconfigure::Server<set_mode::modeConfig>::CallbackType f;
    void point_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void path_callback(const int x_source, const int y_target);
    void delete_point(const int msg);
    void delete_path(const int x_source, const int y_target);
    void drawmap();
    bool getpoint(const Node &point_click, Node &pointback);
    double distance(Node pointa, Node pointb);
    void change_mode(set_mode::modeConfig &comfig);
    void main_loop();
public:
    void run();
    void write_xml();
    GraphTool(ros::NodeHandle nh);
    ~GraphTool();
};

#endif




