/*
 * @Author: your name
 * @Date: 2022-03-13 15:21:33
 * @LastEditTime: 2022-04-16 15:15:11
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/map_plan.cpp
 */
#include "../include/osmmap/map_plan.h"

namespace plan
{

Globalplan::Globalplan(map::centerway::CenterWay *plan_centerways_):plan_centerways(plan_centerways_)
{
    isfindpath = false;
    for(auto it = plan_centerways->Begin(); it != plan_centerways->End(); ++it)
    {
        Planmap *aplan_way = new Planmap(it->second);
        //plan_ways_map.insert(it->second->ID, aplan_way);
        plan_ways_map[it->second->ID] = aplan_way;
    }
    CreatePlanmap();
}

double Globalplan::Centerpoint3d_distance(const map::centerway::CenterPoint3D *a, const map::centerway::CenterPoint3D *b)
{
    double xx = a->x - b->x;
    double yy = a->y - b->y;
    double zz = a->ele - b->ele;
    return std::sqrt(xx*xx + yy*yy + zz*zz);
}

double Globalplan::Centerwaylength(map::centerway::CenterWay3D *centerway_)
{
    double sum = 0;
    for(int i = 0; i < centerway_->length - 1; ++i)
    {
        sum += Centerpoint3d_distance(plan_centerways->Findcenterpoint(centerway_->centernodeline[i]),
                                      plan_centerways->Findcenterpoint(centerway_->centernodeline[i+1]));
    }
    //std::cout << "path " << centerway_->ID << " length is " << sum << std::endl;
    return sum;
}

//当前way->target找下一条way->source
std::vector<int> Globalplan::Findnextcenterway(int plan_centerway_target_)
{
    std::vector<int> nextwayvector;
    for(auto it = plan_centerways->Begin(); it != plan_centerways->End(); ++it)
    {
        double distance_ = Centerpoint3d_distance(plan_centerways->Findcenterpoint(plan_centerway_target_), 
                                                  plan_centerways->Findcenterpoint(it->second->source));
        if(distance_ < 0.1) nextwayvector.push_back(it->second->ID);
    }
    return nextwayvector;
}

void Globalplan::CreatePlanmap()
{
    for(auto it = plan_centerways->Begin(); it != plan_centerways->End(); ++it)
    {
        std::vector<int> nextwaynumbers = Findnextcenterway(it->second->target); 
        if(!nextwaynumbers.empty())
        {
            for(int i = 0; i < nextwaynumbers.size(); ++i)
            {
                plan_ways_map_chain *nextway = new plan_ways_map_chain(plan_centerways->Find(nextwaynumbers[i]));
                nextway->next = plan_ways_map[it->second->ID]->next;
                plan_ways_map[it->second->ID]->next = nextway;
                //std::cout << "connect " << it->second->ID << " and " << nextwaynumbers[i] << std::endl;
            }
        }else{
            //std::cout << it->second->ID << " way can not connect any other way!" << std::endl;
            continue;
        }
    }
}

double Globalplan::Calculateg(int x, int idg)
{
    if(plan_ways_map[idg]->parent == -1)
    {
        return 0;
    }else{
        return plan_ways_map[plan_ways_map[idg]->parent]->G + Centerwaylength(plan_centerways->Find(idg));
    }
}

double Globalplan::Calculateh(int y, int idh)
{
    map::centerway::CenterPoint3D pointy = *plan_centerways->Findcenterpoint(plan_ways_map[y]->plan_ways->target);
    map::centerway::CenterPoint3D pointidh = *plan_centerways->Findcenterpoint(plan_ways_map[idh]->plan_ways->target);
    return std::abs(pointy.x - pointidh.x) + std::abs(pointy.y - pointidh.y) + std::abs(pointy.ele - pointidh.ele);
}

double Globalplan::Calculatef(int idf)
{
    return plan_ways_map[idf]->G + plan_ways_map[idf]->H;
}

int Globalplan::Findleastf(const std::list<int> &listx)
{
    if(listx.empty()) return -1;
    int idres = listx.front();
    for(auto it = listx.begin(); it != listx.end(); ++it)
    {
        //std::cout << plan_ways_map[*it].F << std::endl;
        if(plan_ways_map[*it]->F < plan_ways_map[idres]->F)
        {
            idres = *it;
        }
    }
    return idres;
}

std::vector<int> Globalplan::Getnextnode(int idx)
{
    std::vector<int> res;
    for(auto it = plan_ways_map[idx]->next; it != nullptr; it = it->next)
    {
        //如果该点不在关闭列表,把它加入待选点列表
        if(!Isinlist(it->thisway->ID, closelist))
        {
            res.push_back(it->thisway->ID);
        }
    }
    return res;
}

bool Globalplan::Isinlist(const int x, const std::list<int> &listx)
{
    bool resflag = false;
    for(auto it = listx.begin(); it != listx.end(); ++it)
    {
        if(*it == x)
        {
            resflag = true;
            break;
        }
    }
    return resflag;
}

int Globalplan::Atwhichpoint(const map::centerway::CenterPoint3D &a, const map::centerway::CenterWay3D *centerline_)
{
    for(int i = 0; i < centerline_->length - 1; ++i)
    {
        //a点沿道路点集顺序y轴方向向量#1
        double oyx = plan_centerways->Findcenterpoint(centerline_->centernodeline[i+1])->x - plan_centerways->Findcenterpoint(centerline_->centernodeline[i])->x;
        double oyy = plan_centerways->Findcenterpoint(centerline_->centernodeline[i+1])->y - plan_centerways->Findcenterpoint(centerline_->centernodeline[i])->y;
        //a->p0 #2
        double ap0x = plan_centerways->Findcenterpoint(centerline_->centernodeline[i])->x - a.x;
        double ap0y = plan_centerways->Findcenterpoint(centerline_->centernodeline[i])->y - a.y;
        //a->p1 #3
        double ap1x = plan_centerways->Findcenterpoint(centerline_->centernodeline[i+1])->x - a.x;
        double ap1y = plan_centerways->Findcenterpoint(centerline_->centernodeline[i+1])->y - a.y;
        //#1 * #2 与 #1 * #3异号
        if((oyx*ap0x + oyy*ap0y) * (oyx*ap1x + oyy*ap1y) < 0) return centerline_->centernodeline[i];
    }
    return -1;
}

bool Globalplan::Isintersect(const map::centerway::CenterPoint3D &a_, const map::centerway::CenterPoint3D &b_, 
                             const map::node::Point3D &c_, const map::node::Point3D &d_)
{
    bool flag = false;

    //a->c
    double acx = c_.local_x - a_.x;
    double acy = c_.local_y - a_.y;
    //a->b
    double abx = b_.x - a_.x;
    double aby = b_.y - a_.y;
    //a->d
    double adx = d_.local_x - a_.x;
    double ady = d_.local_y - a_.y;
    //判别CD在AB两侧, AB X AC 与 AB X AD异号---条件1
    double p1 = abx*acy - aby*acx;
    double p2 = abx*ady - aby*adx;

    //c->a
    double cax = -acx;
    double cay = -acy;
    //c->d
    double cdx = d_.local_x - c_.local_x;
    double cdy = d_.local_y - c_.local_y;
    //c->b
    double cbx = b_.x - c_.local_x;
    double cby = b_.y - c_.local_y;
    //判别AB在CD两侧, CD X CA 与 CD X CB异号---条件2
    double p3 = cdx*cay - cdy*cax;
    double p4 = cdx*cby - cdy*cbx;

    //判别条件1和条件2同时成立
    if(p1*p2 < 0 && p3*p4 < 0) flag = true;

    return flag;
}

int Globalplan::Inwhichcenterway(const map::centerway::CenterPoint3D &a, map::node::Node *nodes_, map::way::Way *ways_, map::relation::Relation *relations_)
{
    //1st, 仅依靠该点到centerway两端点的距离和与两端点距离的偏差 < 5% 判断，不准
    /*for(auto it = plan_centerways->Begin(); it != plan_centerways->End(); ++it)
    {
        double atosource = Centerpoint3d_distance(&a, plan_centerways->Findcenterpoint(it->second->source));
        double atotarget = Centerpoint3d_distance(&a, plan_centerways->Findcenterpoint(it->second->target));
        double sourcetotarget = Centerwaylength(it->second);
        if((atosource + atotarget - sourcetotarget) < sourcetotarget * 0.05)
        {
            return it->second->ID;
        }
    }
    return -1;*/

    //2nd, 射线法，好使
    std::vector<map::node::Point3D*> polygon;
    for(auto it = relations_->Begin(); it != relations_->End(); ++it)
    {
        //std::cout << "in relation: " << it->second->ID << std::endl;
        polygon.clear();
        if(it->second->type != map::relation::RelationType::lanelet) continue;
        map::way::Line *left_ = ways_->Find(it->second->leftedge.ID);
        map::way::Line *right_ = ways_->Find(it->second->rightedge.ID);
        double bbox_left = DBL_MAX;
        double bbox_bottom = DBL_MAX;
        double bbox_right = -DBL_MAX;
        double bbox_top = -DBL_MAX;
        for(int i = 0; i < left_->length; ++i)
        {
            map::node::Point3D temp = *nodes_->Find(left_->nodeline[i]);
            polygon.push_back(nodes_->Find(left_->nodeline[i]));
            if(temp.local_x < bbox_left) bbox_left = temp.local_x;
            if(temp.local_x > bbox_right) bbox_right = temp.local_x;
            if(temp.local_y < bbox_bottom) bbox_bottom = temp.local_y;
            if(temp.local_y > bbox_top) bbox_top = temp.local_y;
        }
        for(int i = right_->length - 1; i >= 0; --i)
        {
            map::node::Point3D temp = *nodes_->Find(right_->nodeline[i]);
            polygon.push_back(nodes_->Find(right_->nodeline[i]));
            if(temp.local_x < bbox_left) bbox_left = temp.local_x;
            if(temp.local_x > bbox_right) bbox_right = temp.local_x;
            if(temp.local_y < bbox_bottom) bbox_bottom = temp.local_y;
            if(temp.local_y > bbox_top) bbox_top = temp.local_y;
        }
        if(a.x < bbox_left || a.x > bbox_right || a.y < bbox_bottom || a.y > bbox_top) continue;
        //射线，a->b(b与a等高，但超过多边形的最大右边界10米)
        map::centerway::CenterPoint3D b(bbox_right + 10, a.y);
        //计数穿过多边形次数
        int count = 0;
        //std::cout << it->first << ", polygon size: " << polygon.size() << std::endl;
        for(int i = 0; i < polygon.size(); ++i)
        {
            if(i == polygon.size() - 1)
            {
                if(Isintersect(a, b, *polygon[i], *polygon[0])) count++;
            }else{
                if(Isintersect(a, b, *polygon[i], *polygon[i+1])) count++;
            }
        }
        //std::cout << "count: " << count << std::endl;
        if(count % 2 == 1) return it->second->ID;
    }
    return -1;
}

double Globalplan::Point2edgedistance(const map::centerway::CenterPoint3D &a, map::node::Node *nodes_, map::way::Line *line_, int pathid_)
{
    //int pathid = Inwhichcenterway(a);
    map::centerway::CenterWay3D *path_ = plan_centerways->Find(pathid_);
    int i;//a所在的中心线两端点之一
    bool flag = false;
    //定位目标点a,确定a点y方向
    for(i = 0; i < path_->length - 1; ++i)
    {
        //道路中心线向量#1，i->i+1
        double yy = plan_centerways->Findcenterpoint(path_->centernodeline[i+1])->y - 
                    plan_centerways->Findcenterpoint(path_->centernodeline[i])->y;
        double xx = plan_centerways->Findcenterpoint(path_->centernodeline[i+1])->x - 
                    plan_centerways->Findcenterpoint(path_->centernodeline[i])->x;
            
        //向量#2，a->i+1
        double c1y = plan_centerways->Findcenterpoint(path_->centernodeline[i+1])->y - a.y;
        double c1x = plan_centerways->Findcenterpoint(path_->centernodeline[i+1])->x - a.x;
        //向量#3，a->i
        double c0y = plan_centerways->Findcenterpoint(path_->centernodeline[i])->y - a.y;
        double c0x = plan_centerways->Findcenterpoint(path_->centernodeline[i])->x - a.x;
        //#1 * #2
        double c1 = yy*c1y + xx*c1x;
        //#1 * #3
        double c0 = yy*c0y + xx*c0x;
        if(c0*c1 < 0) 
        {
            flag = true;
            break;
        }
    }
    if(!flag) return -2.0;

    flag = false;
    //a点y方向向量, #4
    int j;
    double oy_y = plan_centerways->Findcenterpoint(path_->centernodeline[i+1])->y - 
                plan_centerways->Findcenterpoint(path_->centernodeline[i])->y;
    double oy_x = plan_centerways->Findcenterpoint(path_->centernodeline[i+1])->x - 
                plan_centerways->Findcenterpoint(path_->centernodeline[i])->x;
    for(j = 0; j < line_->length - 1; ++j)
    {
        //向量#5
        double p1y = nodes_->Find(line_->nodeline[j+1])->local_y - a.y;
        double p1x = nodes_->Find(line_->nodeline[j+1])->local_x - a.x;

        //向量#6
        double p0y = nodes_->Find(line_->nodeline[j])->local_y - a.y;
        double p0x = nodes_->Find(line_->nodeline[j])->local_x - a.x;

        //#4 * #5
        double p1 = oy_y*p1y + oy_x*p1x;
        //#4 * #6
        double p0 = oy_y*p0y + oy_x*p0x;
        if(p1*p0 < 0)
        {
            flag = true;
            break;
        }
    }
    if(!flag) return -1.0;

    //等面积法求距离
    //向量#7
    double l1y = nodes_->Find(line_->nodeline[j+1])->local_y - a.y;
    double l1x = nodes_->Find(line_->nodeline[j+1])->local_x - a.x;

    //向量#8
    double l0y = nodes_->Find(line_->nodeline[j])->local_y - a.y;
    double l0x = nodes_->Find(line_->nodeline[j])->local_x - a.x;

    //#7 x #8 = distance(l1, l0) * 距离
    double d = std::sqrt((l1y-l0y)*(l1y-l0y) + (l1x-l0x)*(l1x-l0x));
    return std::fabs((l1x*l0y - l1y*l0x) / d);
}

//cost: 从x路段的target到y路段的target长度
void Globalplan::Astar(int x, int y)
{
    openlist.clear();
    closelist.clear();
    //放入起点
    openlist.push_back(x);
    plan_ways_map[x]->G = 0;
    plan_ways_map[x]->H = Calculateh(y, x);
    plan_ways_map[x]->F = Calculatef(x);
    while (!openlist.empty())
    {
        int minidnode = Findleastf(openlist);//找到F值最小的点

        openlist.remove(minidnode);//从开启列表中删除
        closelist.push_back(minidnode);//放到关闭列表

        //1,找到下一步待选点
        std::vector<int> candiates = Getnextnode(minidnode);
        for(int i = 0; i < candiates.size(); ++i)
        {
            //2,对某一点，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
            if(!Isinlist(candiates[i], openlist))
            {
                plan_ways_map[candiates[i]]->parent = minidnode;
                plan_ways_map[candiates[i]]->G = Calculateg(x, candiates[i]);
                plan_ways_map[candiates[i]]->H = Calculateh(y, candiates[i]);
                plan_ways_map[candiates[i]]->F = Calculatef(candiates[i]);
                openlist.push_back(candiates[i]);
            }else{
                //3，对某一点，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
                double tempg = Calculateg(x, candiates[i]);
                if(tempg > plan_ways_map[candiates[i]]->G)
                {
                    continue;
                }else{
                    plan_ways_map[candiates[i]]->parent = minidnode;
                    plan_ways_map[candiates[i]]->G = tempg;
                    plan_ways_map[candiates[i]]->F = Calculatef(candiates[i]);
                }
            }
            //如果结束点出现在openlist则搜索成功
            if(Isinlist(y, openlist))
            {
                plan_ways_map[y]->parent = minidnode;
                std::cout << "---------------------------------------------------" << std::endl;
                std::cout << "find!" << std::endl;
                int i = y;
                while(i != -1)
                {
                    plan_path.push_back(i);
                    i = plan_ways_map[i]->parent;
                }
                std::reverse(plan_path.begin(), plan_path.end());
                isfindpath = true;
                return;
            }
        }
    }
    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << "not find" << std::endl;
    return;
}

//H恒等于0的A*
void Globalplan::Dijkstra(int x, int y)
{
    openlist.clear();
    closelist.clear();
    openlist.push_back(x);
    plan_ways_map[x]->G = 0;
    plan_ways_map[x]->H = 0;
    plan_ways_map[x]->F = Calculatef(x);
    while(!openlist.empty())
    {
        int minidnode = Findleastf(openlist);

        openlist.remove(minidnode);
        closelist.push_back(minidnode);

        std::vector<int> candiates = Getnextnode(minidnode);
        for(int i = 0; i < candiates.size(); ++i)
        {
            if(!Isinlist(candiates[i], openlist))
            {
                plan_ways_map[candiates[i]]->parent = minidnode;
                plan_ways_map[candiates[i]]->G = Calculateg(x, candiates[i]);
                plan_ways_map[candiates[i]]->H = 0;
                plan_ways_map[candiates[i]]->F = Calculatef(candiates[i]);
                openlist.push_back(candiates[i]);
            }else{
                double tempg = Calculateg(x, candiates[i]);
                if(tempg > plan_ways_map[candiates[i]]->G)
                {
                    continue;
                }else{
                    plan_ways_map[candiates[i]]->parent = minidnode;
                    plan_ways_map[candiates[i]]->G = tempg;
                    plan_ways_map[candiates[i]]->F = Calculatef(candiates[i]);
                }
            }

            if(Isinlist(y, openlist))
            {
                plan_ways_map[y]->parent = minidnode;
                std::cout << "---------------------------------------------------" << std::endl;
                std::cout << "find!" << std::endl;
                int i = y;
                while(i != -1)
                {
                    plan_path.push_back(i);
                    i = plan_ways_map[i]->parent;
                }
                std::reverse(plan_path.begin(), plan_path.end());
                isfindpath = true;
                return;
            }
        }
    }
    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << "not find" << std::endl;
    return;
}

void Globalplan::Dstar(int x, int y)
{
    //
}

void Globalplan::Reset()
{
    std::cout << "replan ..." << std::endl;
    openlist.clear();
    closelist.clear();
    plan_path.clear();
    isfindpath = false;
    for(auto it = plan_ways_map.begin(); it != plan_ways_map.end(); ++it)
    {
        it->second->F = DBL_MAX;
        it->second->G = DBL_MAX;
        it->second->H = DBL_MAX;
        it->second->parent = -1;
    }
}

bool Globalplan::isReset(int x, int y)
{
    if(plan_path.empty()) return true;
    
    bool flag = true;
    int i = y;
    std::vector<int> plan_path_new;
    while(plan_ways_map[i]->parent != -1)
    {
        plan_path_new.push_back(i);
        i = plan_ways_map[i]->parent;
        if(i == x)
        {
            plan_path_new.push_back(i);
            std::reverse(plan_path_new.begin(), plan_path_new.end());
            plan_path.clear();
            plan_path = plan_path_new;
            flag = false;
            std::cout << "not necessary to replan!" << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;
            break;
        }
    }
    return flag;
}

std::vector<int> Globalplan::run(int x, int y)
{
    //test
    /*map::centerway::CenterPoint3D testa(6.4, 1.2);//6.4, 1.2->220
    int testares = Inwhichcenterway(testa);
    std::cout << "testa is in " << testares << " path" << std::endl;*/

    //replan
    if(isReset(x, y))
    {
        Reset();
        Astar(x, y);
    }

    //feedback
    if(isfindpath)
    {
        std::cout << "start point in path: " << plan_path[0] << 
                     ", goal point in path: " << plan_path[plan_path.size()-1] << std::endl;
        std::cout << "the path as follow: ";
        for(int i = 0; i < plan_path.size(); i++)
        {
            if(i != plan_path.size() - 1)
            {
                //std::cout << plan_path[i] << " " << plan_ways_map[plan_path[i]]->F << " ---> " ;
                std::cout << plan_path[i] << " ---> " ;
            }else{
                std::cout << plan_path[i];
            }
        }
        std::cout << std::endl;
        //std::cout << "cost: " << plan_ways_map[*(--plan_path.end())]->F << std::endl;
    }
    return plan_path;
}


Globalplan::~Globalplan()
{
    //std::cout << "~Globalplan" << std::endl;
    for(auto it = plan_ways_map.begin(); it != plan_ways_map.end(); ++it)
    {
        for(auto iter = it->second->next; iter != nullptr; iter = iter->next)
        {
            delete iter;
        }
        delete it->second;
    }
    plan_ways_map.clear();
}

CBSpline::CBSpline()
{
}
 
 
CBSpline::~CBSpline()
{
}
//======================================================================
// 函数功能： 二次B样条平滑，把给定的点，平滑到B样条曲线上，不增加点的数目
// 输入参数： *pt ：给定点序列，执行完成后，会被替换成新的平滑点
//            Num：点个数
// 返回值：   无返回值
//======================================================================
void CBSpline::TwoOrderBSplineSmooth(std::vector<map::centerway::CenterPoint3D> &pt, int Num)
{
	map::centerway::CenterPoint3D *temp = new map::centerway::CenterPoint3D[Num];
	for(int i = 0; i < Num; i++)
		temp[i] = pt[i];
 
	temp[0].x = 2*temp[0].x - temp[1].x;// 将折线两端点换成延长线上两点
	temp[0].y = 2*temp[0].y - temp[1].y;
    temp[0].ele = 2*temp[0].ele - temp[1].ele;
 
	temp[Num-1].x = 2*temp[Num-1].x - temp[Num-2].x;
	temp[Num-1].y = 2*temp[Num-1].y - temp[Num-2].y;
    temp[Num-1].ele = 2*temp[Num-1].ele - temp[Num-2].ele;
 
	map::centerway::CenterPoint3D NodePt1, NodePt2, NodePt3;
	double t;
    for(int i = 0; i < Num - 2; i++)
	{
		NodePt1 = temp[i]; 
        NodePt2 = temp[i+1]; 
        NodePt3 = temp[i+2];
		if(i == 0){ // 第一段取t=0和t=0.5点   
			t = 0;
			pt[i].x = F02(t)*NodePt1.x + F12(t)*NodePt2.x + F22(t)*NodePt3.x;
			pt[i].y = F02(t)*NodePt1.y + F12(t)*NodePt2.y + F22(t)*NodePt3.y;
            pt[i].ele = F02(t)*NodePt1.ele + F12(t)*NodePt2.ele + F22(t)*NodePt3.ele;
			t = 0.5;
			pt[i+1].x = F02(t)*NodePt1.x + F12(t)*NodePt2.x + F22(t)*NodePt3.x;
			pt[i+1].y = F02(t)*NodePt1.y + F12(t)*NodePt2.y + F22(t)*NodePt3.y;
            pt[i+1].ele = F02(t)*NodePt1.ele + F12(t)*NodePt2.ele + F22(t)*NodePt3.ele;
		}else if(i == Num - 3){ // 最后一段取t=0.5和t=1点
			t = 0.5;
			pt[i+1].x = F02(t)*NodePt1.x + F12(t)*NodePt2.x + F22(t)*NodePt3.x;
			pt[i+1].y = F02(t)*NodePt1.y + F12(t)*NodePt2.y + F22(t)*NodePt3.y;
            pt[i+1].ele = F02(t)*NodePt1.ele + F12(t)*NodePt2.ele + F22(t)*NodePt3.ele;
			t = 1;
			pt[i+2].x = F02(t)*NodePt1.x + F12(t)*NodePt2.x + F22(t)*NodePt3.x;
			pt[i+2].y = F02(t)*NodePt1.y + F12(t)*NodePt2.y + F22(t)*NodePt3.y;
            pt[i+2].ele = F02(t)*NodePt1.ele + F12(t)*NodePt2.ele + F22(t)*NodePt3.ele;
		}else{ // 中间段取t=0.5点
			t = 0.5;
			pt[i+1].x = F02(t)*NodePt1.x + F12(t)*NodePt2.x + F22(t)*NodePt3.x;
			pt[i+1].y = F02(t)*NodePt1.y + F12(t)*NodePt2.y + F22(t)*NodePt3.y;
            pt[i+1].ele = F02(t)*NodePt1.ele + F12(t)*NodePt2.ele + F22(t)*NodePt3.ele;
		}
	}
	delete []temp;
}
 
//================================================================
// 函数功能： 二次B样条拟合,在节点之间均匀插入指定个数点
// 输入参数： *pt ：给定点序列，执行完成后，会被替换成新的数据点
//            Num：节点点个数
//            *InsertNum: 节点之间需要插入的点个数指针 
// 返回值：   无返回值
//
//=================================================================
void CBSpline::TwoOrderBSplineInterpolatePt(std::vector<map::centerway::CenterPoint3D> &pt, int Num, int *InsertNum)
{
	if(pt.size() < 2 || InsertNum == NULL)
    {
        return;
    }
 
	int InsertNumSum = 0; // 计算需要插入的点总数
	for(int i = 0; i < Num - 1; i++)  InsertNumSum += InsertNum[i];
 
	map::centerway::CenterPoint3D *temp = new map::centerway::CenterPoint3D[Num]; // 二次B样条不需要增加点数，需要将首尾点替换掉
	for(int i = 0; i < Num; i++)
		temp[i] = pt[i];
 
	temp[0].x = 2*temp[0].x - temp[1].x; // 将折线两端点换成延长线上两点
	temp[0].y = 2*temp[0].y - temp[1].y;
    temp[0].ele = 2*temp[0].ele - temp[1].ele;
 
	temp[Num-1].x = 2*temp[Num-1].x - temp[Num-2].x;
	temp[Num-1].y = 2*temp[Num-1].y - temp[Num-2].y;
    temp[Num-1].ele = 2*temp[Num-1].ele - temp[Num-2].ele;
 
	//delete []pt;  // 点数由原来的Num个增加到Num+InsertNumSum个，删除旧的存储空间，开辟新的存储空间
    pt.clear();
	//pt = new Node[Num + InsertNumSum];
    pt.resize(Num + InsertNumSum);           
 
	map::centerway::CenterPoint3D NodePt1, NodePt2, NodePt3, NodePt4;// 两节点间均匀插入点，需要相邻两段样条曲线,因此需要四个节点
 
	double t;
	int totalnum = 0;
	for(int i = 0; i < Num - 1; i++) // 每条线段均匀插入点
	{
		if(i == 0){ // 第一段只需计算第一条样条曲线，无NodePt1   
			NodePt2 = temp[i]; 
            NodePt3 = temp[i+1]; 
            NodePt4 = temp[i+2];     
 
			double dt = 0.5/(InsertNum[i] + 1);
			for(int j = 0; j < InsertNum[i] + 1; j++)
			{
				t = 0 + dt*j;
				pt[totalnum].x = F02(t)*NodePt2.x + F12(t)*NodePt3.x + F22(t)*NodePt4.x;
				pt[totalnum].y = F02(t)*NodePt2.y + F12(t)*NodePt3.y + F22(t)*NodePt4.y;
                pt[totalnum].ele = F02(t)*NodePt2.ele + F12(t)*NodePt3.ele + F22(t)*NodePt4.ele;
				totalnum++;
			}
		}else if(i == Num - 2){ // 最后一段只需计算最后一条样条曲线，无NodePt4
			NodePt1 = temp[i-1]; 
            NodePt2 = temp[i]; 
            NodePt3 = temp[i+1];
 
			double dt = 0.5/(InsertNum[i] + 1);
			for(int j = 0; j < InsertNum[i] + 2; j++)
			{
				t = 0.5 + dt*j;
				pt[totalnum].x = F02(t)*NodePt1.x + F12(t)*NodePt2.x + F22(t)*NodePt3.x;
				pt[totalnum].y = F02(t)*NodePt1.y + F12(t)*NodePt2.y + F22(t)*NodePt3.y;
                pt[totalnum].ele = F02(t)*NodePt1.ele + F12(t)*NodePt2.ele + F22(t)*NodePt3.ele;
				totalnum++;
			}
		}else{
			NodePt1 = temp[i-1];
            NodePt2 = temp[i]; 
            NodePt3 = temp[i+1]; 
            NodePt4 = temp[i+2]; //NodePt1,2,3计算第一条曲线，NodePt2,3,4计算第二条曲线
 
			int LeftInsertNum, RightInsertNum;// 计算线段间左右曲线段上分别需要插入的点数
			double rightoffset = 0; // 左边曲线段从t=0.5开始，又边曲线段从t=rightoffset开始
			double Leftdt = 0, Rightdt = 0; // 左右曲线取点t步长
			if(InsertNum[i] == 0){
				LeftInsertNum = 0;
				RightInsertNum = 0;
			}else if(InsertNum[i] % 2 == 1){// 插入点数为奇数，左边曲线段插入点个数比右边多一点
				RightInsertNum = InsertNum[i]/2;
				LeftInsertNum = RightInsertNum + 1;
				Leftdt = 0.5/(LeftInsertNum);
				Rightdt = 0.5/(RightInsertNum + 1);
				rightoffset = Rightdt;
			}else{ // 插入点数为偶数，左右边曲线段插入个数相同
				RightInsertNum = InsertNum[i]/2;
				LeftInsertNum = RightInsertNum;
				Leftdt = 0.5/(LeftInsertNum + 0.5);
				Rightdt = 0.5/(RightInsertNum + 0.5);
				rightoffset = Rightdt/2;
			}
 
			for(int j = 0; j < LeftInsertNum + 1; j++)
			{
				t = 0.5 + Leftdt*j;
				pt[totalnum].x = F02(t)*NodePt1.x + F12(t)*NodePt2.x + F22(t)*NodePt3.x;
				pt[totalnum].y = F02(t)*NodePt1.y + F12(t)*NodePt2.y + F22(t)*NodePt3.y;
                pt[totalnum].ele = F02(t)*NodePt1.ele + F12(t)*NodePt2.ele + F22(t)*NodePt3.ele;
				totalnum++;
			}
 
			for(int j = 0; j < RightInsertNum; j++)
			{				
				t = rightoffset + Rightdt*j;
				pt[totalnum].x = F02(t)*NodePt2.x + F12(t)*NodePt3.x + F22(t)*NodePt4.x;
				pt[totalnum].y = F02(t)*NodePt2.y + F12(t)*NodePt3.y + F22(t)*NodePt4.y;
                pt[totalnum].ele = F02(t)*NodePt2.ele + F12(t)*NodePt3.ele + F22(t)*NodePt4.ele;
				totalnum++;
			}
		}
	}
	delete []temp;
	Num = Num + InsertNumSum;
 
}
//================================================================
// 函数功能： 二次样条基函数
//
//================================================================
double CBSpline::F02(double t)
{
	return 0.5*(t - 1)*(t - 1);
}
double CBSpline::F12(double t)
{
	return 0.5*(-2*t*t + 2*t + 1);
}
double CBSpline::F22(double t)
{
	return 0.5*t*t;
}
//========================================================================
// 函数功能： 三次B样条平滑，把给定的点，平滑到B样条曲线上，不增加点的数目
// 输入参数： *pt ：给定点序列，执行完成后，会被替换成新的平滑点
//            Num：点个数
// 返回值：   无返回值
//
//========================================================================
void CBSpline::ThreeOrderBSplineSmooth(std::vector<map::centerway::CenterPoint3D> &pt, int Num)
{
	map::centerway::CenterPoint3D *temp = new map::centerway::CenterPoint3D[Num + 2];
	for(int i = 0; i < Num; i++)
		temp[i+1] = pt[i];
 
	temp[0].x = 2*temp[1].x - temp[2].x; // 将折线延长线上两点加入作为首点和尾点
	temp[0].y = 2*temp[1].y - temp[2].y;
    temp[0].ele = 2*temp[1].ele - temp[2].ele;
 
	temp[Num+1].x = 2*temp[Num].x - temp[Num-1].x;
	temp[Num+1].y = 2*temp[Num].y - temp[Num-1].y;
    temp[Num+1].ele = 2*temp[Num].ele - temp[Num-1].ele;
 
	map::centerway::CenterPoint3D NodePt1, NodePt2, NodePt3, NodePt4;
	double t;
	for(int i = 0; i < Num - 1; i++)
	{
		NodePt1 = temp[i]; 
        NodePt2 = temp[i+1]; 
        NodePt3 = temp[i+2]; 
        NodePt4 = temp[i+3];
 
		if(i == Num - 4){ // 最后一段取t=0.5和t=1点
			t = 0;
			pt[i].x = F03(t)*NodePt1.x + F13(t)*NodePt2.x + F23(t)*NodePt3.x + F33(t)*NodePt4.x;
			pt[i].y = F03(t)*NodePt1.y + F13(t)*NodePt2.y + F23(t)*NodePt3.y + F33(t)*NodePt4.y;
            pt[i].ele = F03(t)*NodePt1.ele + F13(t)*NodePt2.ele + F23(t)*NodePt3.ele + F33(t)*NodePt4.ele;
			t = 1;
			pt[i+1].x = F03(t)*NodePt1.x + F13(t)*NodePt2.x + F23(t)*NodePt3.x + F33(t)*NodePt4.x;
			pt[i+1].y = F03(t)*NodePt1.y + F13(t)*NodePt2.y + F23(t)*NodePt3.y + F33(t)*NodePt4.y;
            pt[i+1].ele = F03(t)*NodePt1.ele + F13(t)*NodePt2.ele + F23(t)*NodePt3.ele + F33(t)*NodePt4.ele;
		}else{ // 中间段取t=0.5点
			t = 0;
			pt[i].x = F03(t)*NodePt1.x + F13(t)*NodePt2.x + F23(t)*NodePt3.x + F33(t)*NodePt4.x;
			pt[i].y = F03(t)*NodePt1.y + F13(t)*NodePt2.y + F23(t)*NodePt3.y + F33(t)*NodePt4.y;
            pt[i].ele = F03(t)*NodePt1.ele + F13(t)*NodePt2.ele + F23(t)*NodePt3.ele + F33(t)*NodePt4.ele;
		}
	}
	delete []temp;
}
 
//================================================================
// 函数功能： 三次B样条拟合,在节点之间均匀插入指定个数点
// 输入参数： *pt ：给定点序列，执行完成后，会被替换成新的数据点
//            Num：节点点个数
//            *InsertNum: 节点之间需要插入的点个数指针 
// 返回值：   无返回值
//
//=================================================================
void CBSpline::ThreeOrderBSplineInterpolatePt(std::vector<map::centerway::CenterPoint3D> &pt, int Num, int *InsertNum)
{
	if(pt.size() < 2 || InsertNum == NULL) return;
 
	int InsertNumSum = 0;// 计算需要插入的点总数
	for(int i = 0; i < Num - 1; i++)  InsertNumSum += InsertNum[i];
 
	map::centerway::CenterPoint3D *temp = new map::centerway::CenterPoint3D[Num + 2];
	for(int i = 0; i < Num; i++)
		temp[i + 1] = pt[i];
 
	temp[0].x = 2*temp[1].x - temp[2].x;// 将折线延长线上两点加入作为首点和尾点
	temp[0].y = 2*temp[1].y - temp[2].y;
    temp[0].ele = 2*temp[1].ele - temp[2].ele;
 
	temp[Num+1].x = 2*temp[Num].x - temp[Num-1].x;
	temp[Num+1].y = 2*temp[Num].y - temp[Num-1].y;
    temp[Num+1].ele = 2*temp[Num].ele - temp[Num-1].ele;
 
	map::centerway::CenterPoint3D NodePt1, NodePt2, NodePt3, NodePt4;
	double t;
 
	//delete []pt;// 点数由原来的Num个增加到Num+InsertNumSum个，删除旧的存储空间，开辟新的存储空间
    pt.clear();
	//pt = new Node[Num + InsertNumSum];
    pt.resize(Num + InsertNumSum);           
 
	int totalnum = 0;
	for(int i = 0; i < Num - 1; i++) // 每条线段均匀插入点
	{
		NodePt1 = temp[i]; 
        NodePt2 = temp[i+1]; 
        NodePt3 = temp[i+2]; 
        NodePt4 = temp[i+3];
		double dt = 1.0/(InsertNum[i] + 1);
 
		for(int j = 0; j < InsertNum[i] + 1; j++)
		{
			t = dt*j;
			pt[totalnum].x = F03(t)*NodePt1.x + F13(t)*NodePt2.x + F23(t)*NodePt3.x + F33(t)*NodePt4.x;
			pt[totalnum].y = F03(t)*NodePt1.y + F13(t)*NodePt2.y + F23(t)*NodePt3.y + F33(t)*NodePt4.y;
            pt[totalnum].ele = F03(t)*NodePt1.ele + F13(t)*NodePt2.ele + F23(t)*NodePt3.ele + F33(t)*NodePt4.ele;
			totalnum++;
		}
 
		if(i == Num - 2){ // 最后一个尾点
			t = 1;
			pt[totalnum].x = F03(t)*NodePt1.x + F13(t)*NodePt2.x + F23(t)*NodePt3.x + F33(t)*NodePt4.x;
			pt[totalnum].y = F03(t)*NodePt1.y + F13(t)*NodePt2.y + F23(t)*NodePt3.y + F33(t)*NodePt4.y;
            pt[totalnum].ele = F03(t)*NodePt1.ele + F13(t)*NodePt2.ele + F23(t)*NodePt3.ele + F33(t)*NodePt4.ele;
			totalnum++;
		}
	}
 
	delete []temp;
	Num = Num + InsertNumSum;
 
}

//================================================================
// 函数功能： 三次样条基函数
//
//================================================================
double CBSpline::F03(double t)
{
	return 1.0/6*(-t*t*t + 3*t*t - 3*t + 1);
}
double CBSpline::F13(double t)
{
	return 1.0/6*(3*t*t*t - 6*t*t + 4);
}
double CBSpline::F23(double t)
{
	return 1.0/6*(-3*t*t*t + 3*t*t + 3*t + 1);
}
double CBSpline::F33(double t)
{
	return 1.0/6*t*t*t;
}



};//namespace plan




