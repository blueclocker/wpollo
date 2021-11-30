/*
 * @Author: your name
 * @Date: 2021-11-22 13:14:01
 * @LastEditTime: 2021-11-26 16:00:41
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/include/AStar.h
 */
#ifndef ASTAR_H_
#define ASTAR_H_

#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <math.h>
#include <algorithm>

struct Node
{
    int x,y;//坐标
    int g;//起点到当前点代价
    int h;//当前点到目标点代价
    int f;//代价估计值g+h
    Node* father;
    Node(int a, int b):x(a),y(b),g(0),h(0),f(0),father(nullptr)
    {
        /*this->x = a;
        this->y = b;
        this->g = 0;
        this->h = 0;
        this->f = 0;
        this->father = nullptr;*/
    }
    Node(int a, int b, Node* father_):x(a),y(b),g(0),h(0),f(0),father(father_)
    {
        /*this->x = a;
        this->y = b;
        this->g = 0;
        this->h = 0;
        this->f = 0;
        this->father = father_;*/
    }
};

class AStar
{
private:
    static const int wightw = 10;//水平移动代价
    static const int wighth = 14;//斜向移动代价
    bool isIgnoreCorner;//是否忽略斜角穿墙
public:
    std::list<Node*> openlist;//开启列表
    std::list<Node*> closelist;//关闭列表
    std::vector<std::vector<int>> map;//地图, 1障碍, 0可通行
    Node* start_node;//起点
    Node* end_node;//终点
    bool init(Node* start_, Node* end_, const std::vector<std::vector<int>> &map_);//初始化，设置起点、终点、地图
    int calcG(const Node* last_node, const Node* current_node);//计算G
    int calcH(const Node* current_node);//计算H
    int calcF(const Node* current_node);//计算G+H
    Node* getLeastFnode();//从开启列表中返回F值最小的节点
    bool isCanReach(const Node* current_node, const Node* next_node);//判断当前点的下一点能否到达
    bool isInCloselist(const Node* current_node);//判断当前点是否在关闭列表
    bool isInOpenlist(const Node* current_node);//判断当前点是否在开启列表
    std::vector<Node*> getNextNode(const Node* current_node);//获取当前点的周围八个点
    virtual bool search();//搜索是否有解
    
    virtual std::list<Node*> solve();//A*输出接口
    AStar(Node* start, Node* end, const std::vector<std::vector<int>> &map_, const bool isIgnoreCorner_ = false);
    AStar();
    ~AStar();
};



#endif