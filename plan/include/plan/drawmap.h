/*
 * @Author: your name
 * @Date: 2021-11-30 14:32:45
 * @LastEditTime: 2021-12-03 16:02:39
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/include/plan/drawmap.cpp
 */

#ifndef DRAWMAP_H_
#define DRAWMAP_H_

#include <plan/tinyxml.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

struct mapinfo
{
    int id;
    double x;
    double y;
};

struct connect
{
    int source;
    int target;
    double distance;//source 与 target 距离
    double angle;//source 与 targrt 夹角
};


class GlobalMap
{
private:
    std::string map_path;
    bool isread;//true->read, false->write
    ros::NodeHandle n;
    ros::Time currenttime;
    ros::Publisher pub_globalmap;
    ros::Publisher pub_mapnode;
    ros::Subscriber sub_globalmap;
    visualization_msgs::MarkerArray maps;
    //visualization_msgs::MarkerArray points;
    std::vector<mapinfo> globalmap_info;
    std::vector<connect> connection;
    std::vector<std::string> key_name;
    std::vector<std::string> key_type;
    std::vector<std::string> key_for;
    std::vector<std::string> key_id;
    TiXmlDocument doc;
    TiXmlElement *head_write;
    int count;
    //TiXmlDocument doc;
    void read_xml();
    void drawmap();
    void map_publish();
    void mapcallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void write_xml();
public:
    GlobalMap(ros::NodeHandle &n_, const std::string file_path, const bool flag);
    ~GlobalMap();
};



#endif