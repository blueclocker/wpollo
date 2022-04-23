/*
 * @Author: your name
 * @Date: 2022-03-13 15:21:15
 * @LastEditTime: 2022-04-23 10:35:00
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
    double Centerpoint3d_distance(const map::centerway::CenterPoint3D *a, const map::centerway::CenterPoint3D *b) const;
    double Centerwaylength(const map::centerway::CenterWay3D *centerway_) const;
    std::vector<int> Findnextcenterway(const int plan_centerway_target_) const;
    void CreatePlanmap();
    double Calculateg(const int x, const int idg) const;
    double Calculateh(const int y, const int idh) const;
    double Calculatef(const int idf) const;
    int Findleastf(const std::list<int> &listx) const;
    std::vector<int> Getnextnode(const int idx) const;
    bool Isinlist(const int x, const std::list<int> &listx) const;
    //int Inwhichcenterway(map::centerway::CenterPoint3D a);
    //int Atwhichpoint(const map::centerway::CenterPoint3D &a, const map::centerway::CenterWay3D *centerline_);
    bool Isintersect(const map::centerway::CenterPoint3D &a_, const map::centerway::CenterPoint3D &b_, 
                     const map::node::Point3D &c_, const map::node::Point3D &d_) const;
    bool isReset(const int x, const int y);
    void Astar(const int x, const int y);
    void Dijkstra(const int x, const int y);
    void Dstar(int x, int y);
public:
    Globalplan(map::centerway::CenterWay *plan_centerways_);
    ~Globalplan();
    std::vector<int> run(const int x, const int y);//返回plan_path
    int Inwhichcenterway(const map::centerway::CenterPoint3D &a, const map::node::Node *nodes_, const map::way::Way *ways_, const map::relation::Relation *relations_) const;
    int Atwhichpoint(const map::centerway::CenterPoint3D &a, const map::centerway::CenterWay3D *centerline_) const;
    double Point2edgedistance(const map::centerway::CenterPoint3D &a, const map::node::Node *nodes_, map::way::Line *line_, int pathid_) const;
    void Reset();//重置邻接表，openlist，closelist等
};

class CBSpline
{
public:
	CBSpline();
	~CBSpline();
 
	void TwoOrderBSplineSmooth(std::vector<map::centerway::CenterPoint3D> &pt,int Num);
	void TwoOrderBSplineInterpolatePt(std::vector<map::centerway::CenterPoint3D> &pt,int Num,int *InsertNum);
	double F02(double t);
	double F12(double t);
	double F22(double t);
 
	void ThreeOrderBSplineSmooth(std::vector<map::centerway::CenterPoint3D> &pt,int Num);
	void ThreeOrderBSplineInterpolatePt(std::vector<map::centerway::CenterPoint3D> &pt,int Num,int *InsertNum);
	double F03(double t);
	double F13(double t);
	double F23(double t);
	double F33(double t);
};

};//namespace plan

#endif