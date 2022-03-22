/*
 * @Author: your name
 * @Date: 2022-03-13 15:21:15
 * @LastEditTime: 2022-03-20 21:55:00
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_plan.h
 */
#ifndef MAP_PLAN_H_
#define MAP_PLAN_H_

#include "map_node.h"
#include "map_way.h"
#include "map_relation.h"
#include "centerway.h"
#include <algorithm>
#include <cfloat>
#include <vector>
#include <list>

namespace plan
{
struct plan_ways_map_chain
{
    map::centerway::CenterWay3D *thisway;
    plan_ways_map_chain *next;
    plan_ways_map_chain() {}
    plan_ways_map_chain(map::centerway::CenterWay3D *x)
    {
        thisway = x;
        next = nullptr;
    }
};

struct Planmap
{
    map::centerway::CenterWay3D *plan_ways;
    int parent;
    double F, G, H;
    plan_ways_map_chain *next;
    Planmap() {}
    Planmap(map::centerway::CenterWay3D *x)
    {
        plan_ways = x;
        parent = -1;
        F = DBL_MAX;
        G = DBL_MAX;
        H = DBL_MAX;
        next = nullptr;
    }
};

    
class Globalplan
{
private:
    std::unordered_map<int, Planmap*> plan_ways_map;
    map::centerway::CenterWay *plan_centerways;
    std::list<int> openlist;
    std::list<int> closelist;
    std::vector<int> plan_path;
    bool isfindpath;
    double Centerpoint3d_distance(map::centerway::CenterPoint3D *a, map::centerway::CenterPoint3D *b);
    double Centerwaylength(map::centerway::CenterWay3D *centerway_);
    std::vector<int> Findnextcenterway(int plan_centerway_target_);
    void CreatePlanmap();
    double Calculateg(int x, int idg);
    double Calculateh(int y, int idh);
    double Calculatef(int idf);
    int Findleastf(const std::list<int> &listx);
    std::vector<int> Getnextnode(int idx);
    bool Isinlist(const int x, const std::list<int> &listx);
    //int Inwhichcenterway(map::centerway::CenterPoint3D a);
    void Astar(int x, int y);
public:
    Globalplan(map::centerway::CenterWay *plan_centerways_);
    ~Globalplan();
    std::vector<int> run(int x, int y);//返回plan_path
    int Inwhichcenterway(map::centerway::CenterPoint3D a);
    void Reset();//重置邻接表，openlist，closelist等
};





};//namespace plan

#endif