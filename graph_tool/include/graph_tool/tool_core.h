/*
 * @Author: your name
 * @Date: 2021-12-28 11:22:46
 * @LastEditTime: 2022-01-03 22:19:02
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int8.h>
#include "graph_tool/SetPath.h"
#include "graph_tool/PathPoint.h"

struct Node
{
    int id_node;
    double x_node;
    double y_node;
};

struct Line
{
    int source_line;
    int target_line;
    double distance_line;
    double angel_line;
};

class GraphTool
{
private:
    ros::NodeHandle n;
    ros::Subscriber point_sub;
    ros::Subscriber line_sub;
    ros::Subscriber delete_point_sub;
    ros::Subscriber delete_line_sub;
    ros::Publisher point_pub;
    ros::Publisher path_pub;
    visualization_msgs::MarkerArray points;
    visualization_msgs::MarkerArray paths;
    std::vector<Node> nodewithid;
    std::vector<Line> linewithpoints;
    std::vector<std::string> key_name;
    std::vector<std::string> key_type;
    std::vector<std::string> key_for;
    std::vector<std::string> key_id;
    std::string map_path_;
    std::string map_name_;
    TiXmlDocument doc;
    TiXmlElement *head_write;
    bool isnew_;
    int count;
    int linenumber;
    void point_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void path_callback(const graph_tool::PathPoint::ConstPtr &msg);
    void delete_point(const std_msgs::Int8::ConstPtr &msg);
    void delete_line(const graph_tool::PathPoint::ConstPtr &msg);
    void drawmap();
public:
    void write_xml();
    GraphTool(ros::NodeHandle nh);
    ~GraphTool();
};

#endif




