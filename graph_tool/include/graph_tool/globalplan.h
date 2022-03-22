/*
 * @Author: your name
 * @Date: 2022-01-11 15:15:55
 * @LastEditTime: 2022-03-13 13:14:23
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/include/graph_tool/globalplan.h
 */
#ifndef GOLBALPLAN_H_
#define GLOBALPLAN_H_

#include <iostream>
#include <cfloat>
#include <vector>
#include <list>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "tinyxml.h"

struct Node
{
    int id_node;
    double x_node;
    double y_node;
    Node(int x=0)
    {
        id_node = x;
        x_node = 0;
        y_node = 0;
    };
    Node(double pos_x, double pos_y)
    {
        id_node = -1;
        x_node = pos_x;
        y_node = pos_y;
    };
    bool operator==(const Node &a)
    {
        //std::cout << "operate ==" << std::endl;
        return(this->id_node == a.id_node && this->x_node == a.x_node && this->y_node == a.y_node);
    }
};

struct Path
{
    int source_path;
    int target_path;
    double distance_path;
    double angel_path;
    Path& operator=(const Path &a)
    {
        this->source_path = a.source_path;
        this->target_path = a.target_path;
        this->distance_path = a.distance_path;
        this->angel_path = a.angel_path;
    }
};

struct mapnode
{
    Node data;
    bool isvisit;
    mapnode* next;
    mapnode(Node x)
    {
        data = x;
        isvisit = false;
        next = nullptr;
    };
};

struct Astarmapnode
{
    int id;//点编号
    double F, G, H;//A*代价
    int parent;//父节点
};

class GlobalPlan
{
private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher path_pub;
    std::vector<Node> nodewithid;
    Node* hashnode;
    std::vector<Path> pathwithpoints;
    visualization_msgs::MarkerArray map_marker;
    visualization_msgs::MarkerArray path_marker;
    std::string map_path_;
    std::string map_name_;
    int start_position_;
    int end_position_;
    mapnode** map;
    Astarmapnode* Astarmap;
    int maxnodenumber;
    std::vector<int> pathres;
    std::list<int> openlist;
    std::list<int> closelist;
    bool isfindpath;
    //std::vector<std::vector<Node> > smoothpathnode;
    std::vector<Node> smoothpathnode;
    void createmap();
    bool isnext(const int a, const int b);
    void Dijkstra(int x, int y);
    void DFS(int x, int y);
    void DFS_pro(int x, int y);
    void Astar(int x, int y);
    int findleastf(const std::list<int> &listx);
    double calculateg(int x, int idg);
    double calculateh(int y, int idh);
    double calculatef(int idf);
    std::vector<int> getnextnode(int idx);
    bool isinlist(const int x, const std::list<int> &listx);
    Node findnode(const int id_);
    int findnext(const int id_, const int finalid);
    double distance(const Node a, const Node b);
    bool isturn(const Path &a, const Path &b);
    Path findpath(int a, int b);
    void findsmoothpath();
    void pathvisual();
public:
    void run();
    GlobalPlan(ros::NodeHandle nh);
    ~GlobalPlan();
};

//贝塞尔插值
class Bezier 
{
private:
    int ctrl_num_;  //控制点数量
    int order_;     //阶数，表示曲线为几阶曲线，为控制点数-1
    int knot_num_;  //节点数是分布在0-1之间有多少个点
    std::vector<Node> control_points_;
    std::vector<Node> path_points_;
public:
    Bezier() = default;
    const std::vector<Node>& CalculateSpline(
        const std::vector<Node>& control_points,
        const int& points_number);
    void CalculateSecond();
    void CalculateCubic();
    void Calculate(int Num);
    int C_nm(int n, int m);
};

class CBSpline
{
public:
	CBSpline();
	~CBSpline();
 
	void TwoOrderBSplineSmooth(std::vector<Node> &pt,int Num);
	void TwoOrderBSplineInterpolatePt(std::vector<Node> &pt,int Num,int *InsertNum);
	double F02(double t);
	double F12(double t);
	double F22(double t);
 
	void ThreeOrderBSplineSmooth(std::vector<Node> &pt,int Num);
	void ThreeOrderBSplineInterpolatePt(std::vector<Node> &pt,int Num,int *InsertNum);
	double F03(double t);
	double F13(double t);
	double F23(double t);
	double F33(double t);
};

#endif