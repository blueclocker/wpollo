/*
 * @Author: your name
 * @Date: 2022-03-13 15:21:15
 * @LastEditTime: 2022-10-03 16:19:07
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
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
    map::centerway::CenterWay3D *thisway_;
    plan_ways_map_chain *next_;
    plan_ways_map_chain() {}
    plan_ways_map_chain(map::centerway::CenterWay3D *x)
    {
        thisway_ = x;
        next_ = nullptr;
    }
};

struct Planmap
{
    map::centerway::CenterWay3D *plan_ways_;
    int parent_;
    double F_, G_, H_;
    plan_ways_map_chain *next_;
    Planmap() {}
    Planmap(map::centerway::CenterWay3D *x)
    {
        plan_ways_ = x;
        parent_ = -1;
        F_ = DBL_MAX;
        G_ = DBL_MAX;
        H_ = DBL_MAX;
        next_ = nullptr;
    }
};

    
class Globalplan
{
private:
    std::unordered_map<int, Planmap*> plan_ways_map_;
    map::centerway::CenterWay *plan_centerways_;
    std::list<int> openlist_;
    std::list<int> closelist_;
    std::vector<int> plan_path_;
    bool isfindpath_;
    double CenterPoint3dDistance(const map::centerway::CenterPoint3D *a, const map::centerway::CenterPoint3D *b) const;
    double CenterwayLength(const map::centerway::CenterWay3D *centerway) const;
    std::vector<int> FindNextCenterway(const int plan_centerway_target) const;
    void CreatePlanmap();
    double Calculateg(const int x, const int idg) const;
    double Calculateh(const int y, const int idh) const;
    double Calculatef(const int idf) const;
    int Findleastf(const std::list<int> &listx) const;
    std::vector<int> GetNextNode(const int idx) const;
    bool IsinList(const int x, const std::list<int> &listx) const;
    //int Inwhichcenterway(map::centerway::CenterPoint3D a);
    //int Atwhichpoint(const map::centerway::CenterPoint3D &a, const map::centerway::CenterWay3D *centerline_);
    bool IsIntersect(const map::centerway::CenterPoint3D &a, const map::centerway::CenterPoint3D &b, 
                     const map::node::Point3D &c, const map::node::Point3D &d) const;
    bool IsReset(const int x, const int y);
    void Astar(const int x, const int y);
    void Dijkstra(const int x, const int y);
    void Dstar(int x, int y);
public:
    Globalplan(map::centerway::CenterWay *plan_centerways);
    ~Globalplan();
    void DeleteChain(plan_ways_map_chain *root);
    std::vector<int> Run(const int x, const int y);//返回plan_path
    int InWhichCenterway(const map::centerway::CenterPoint3D &a, const map::node::Node *nodes, const map::way::Way *ways, const map::relation::Relation *relations) const;
    std::vector<int> LocateLanelets(const map::centerway::CenterPoint3D &a, const map::node::Node *nodes, const map::way::Way *ways, const map::relation::Relation *relations) const;
    int AtWhichPoint(const map::centerway::CenterPoint3D &a, const map::centerway::CenterWay3D *centerline) const;
    double Point2EdgeDistance(const map::centerway::CenterPoint3D &a, const map::node::Node *nodes, map::way::Line *line, int pathid) const;
    void Reset();//重置邻接表，openlist，closelist等
    std::vector<int> FindNextLanes(const int id) const;
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