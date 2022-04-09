/*
 * @Author: your name
 * @Date: 2022-03-31 15:44:49
 * @LastEditTime: 2022-04-07 16:57:55
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/plan/include/plan/DStar.h
 */
#ifndef DSTAR_H_
#define DSTAR_H_

#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <math.h>
#include <algorithm>

struct DNode
{
    int x,y;//坐标
    int H;//当前点到目标点的代价
    int K;//表示该点被置于Openlist后，该点到目标点的最小代价H(x)
    int nextx, nexty;
    //DNode *next;
    DNode() {}
    DNode(int x_, int y_)
    {
        x = x_;
        y = y_;
        H = INT16_MAX;
        K = INT16_MAX;
        nextx = -1;
        nexty = -1;
        //next = nullptr;
    }
    DNode(const DNode &a)
    {
        x = a.x;
        y = a.y;
        H = a.H;
        K = a.K;
        //next = a.next;
        nextx = a.nextx;
        nexty = a.nexty;
    }
    DNode& operator=(const DNode &a)
    {
        this->x = a.x;
        this->y = a.y;
        this->H = a.H;
        this->K = a.K;
        this->nextx = a.nextx;
        this->nexty = a.nexty;
        return *this;
    }
    bool operator==(const DNode &a)
    {
        return (this->x == a.x && this->y == a.y);
    }
};


class DStar
{
private:
    static const int wightw = 10;//水平移动代价
    static const int wighth = 14;//斜向移动代价
    DNode start;
    DNode goal;
    std::vector<DNode> openlist;//依据K值由小到大进行排序的优先队列
    std::vector<DNode> closelist;
    std::vector<std::vector<int>> map;//地图, 1障碍, 0可通行
    std::vector<std::vector<DNode>> Dstarmap;
    void sortOpenlist();
    int cost(const DNode &a, const DNode &b) const;
    void insert(const DNode &a, int h_);
    int calculateH(const DNode &a) const;
    bool isInList(const DNode &a, const std::vector<DNode> &list_) const;
    void deleteInList(const DNode &a, std::vector<DNode> &list_);
    bool isCanReach(const DNode &current_node, const DNode &next_node) const;//判断当前点的下一点能否到达
    std::vector<DNode> getNextNode(const DNode &a) const;//获取当前点的周围八个点
    std::vector<DNode> getNextNodeReplan(const DNode &a) const;
    bool Dijstra();
    int processState();
    //bool isCanReach(const DNode a) const;
public:
    DStar(std::vector<std::vector<int>> map_);
    ~DStar();
    void changeMap(int x_, int y_);
    std::vector<DNode> rePlan();
    void RePlan();
    void DstarRun(DNode &a, DNode &b);
    std::vector<DNode> findPath();
    bool isCanReachGoal(const DNode &a) const;
};







#endif