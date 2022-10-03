/*
 * @Author: your name
 * @Date: 2022-03-13 15:21:33
 * @LastEditTime: 2022-10-03 19:07:16
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/plan/map_plan.cpp
 */
#include "../include/osmmap/map_plan.h"

namespace plan
{

Globalplan::Globalplan(map::centerway::CenterWay *plan_centerways):plan_centerways_(plan_centerways)
{
    isfindpath_ = false;
    for(auto it = plan_centerways->Begin(); it != plan_centerways->End(); ++it)
    {
        Planmap *aplan_way = new Planmap(it->second);
        //plan_ways_map.insert(it->second->ID, aplan_way);
        plan_ways_map_[it->second->ID_] = aplan_way;
    }
    CreatePlanmap();
}

double Globalplan::CenterPoint3dDistance(const map::centerway::CenterPoint3D *a, const map::centerway::CenterPoint3D *b) const 
{
    double xx = a->x_ - b->x_;
    double yy = a->y_ - b->y_;
    double zz = a->ele_ - b->ele_;
    return std::sqrt(xx*xx + yy*yy + zz*zz);
}

double Globalplan::CenterwayLength(const map::centerway::CenterWay3D *centerway) const 
{
    double sum = 0;
    for(int i = 0; i < centerway->length_ - 1; ++i)
    {
        sum += CenterPoint3dDistance(plan_centerways_->FindCenterPoint(centerway->centernodeline_[i]),
                                     plan_centerways_->FindCenterPoint(centerway->centernodeline_[i+1]));
    }
    //std::cout << "path " << centerway_->ID << " length is " << sum << std::endl;
    return sum;
}

//当前way->target找下一条way->source
std::vector<int> Globalplan::FindNextCenterway(const int plan_centerway_target) const 
{
    std::vector<int> nextwayvector;
    for(auto it = plan_centerways_->Begin(); it != plan_centerways_->End(); ++it)
    {
        double distance = CenterPoint3dDistance(plan_centerways_->FindCenterPoint(plan_centerway_target), 
                                                plan_centerways_->FindCenterPoint(it->second->source_));
        if(distance < 0.1) nextwayvector.push_back(it->second->ID_);
    }
    return nextwayvector;
}

void Globalplan::CreatePlanmap()
{
    for(auto it = plan_centerways_->Begin(); it != plan_centerways_->End(); ++it)
    {
        std::vector<int> nextwaynumbers = FindNextCenterway(it->second->target_); 
        if(!nextwaynumbers.empty())
        {
            for(int i = 0; i < nextwaynumbers.size(); ++i)
            {
                plan_ways_map_chain *nextway = new plan_ways_map_chain(plan_centerways_->Find(nextwaynumbers[i]));
                nextway->next_ = plan_ways_map_[it->second->ID_]->next_;
                plan_ways_map_[it->second->ID_]->next_ = nextway;
                //std::cout << "connect " << it->second->ID << " and " << nextwaynumbers[i] << std::endl;
            }
        }else{
            //std::cout << it->second->ID << " way can not connect any other way!" << std::endl;
            continue;
        }
    }
}

double Globalplan::Calculateg(const int x, const int idg) const 
{
    if(plan_ways_map_.at(idg)->parent_ == -1)
    {
        return 0;
    }else{
        double g;
        g = plan_ways_map_.at(plan_ways_map_.at(idg)->parent_)->G_ + CenterwayLength(plan_centerways_->Find(idg));
        //加换道损失，改进H计算方式
        //如果是相邻路段，则在H上附加换道损失-----换道
        if(plan_centerways_->IsNeighbor(idg, plan_ways_map_.at(idg)->parent_))
        {
            g += 10;//换道损失暂定10m
            //std::cout << "change lane" << std::endl;
        }
        return g;
    }
}

double Globalplan::Calculateh(const int y, const int idh) const 
{
    map::centerway::CenterPoint3D pointy = *plan_centerways_->FindCenterPoint(plan_ways_map_.at(y)->plan_ways_->target_);
    map::centerway::CenterPoint3D pointidh = *plan_centerways_->FindCenterPoint(plan_ways_map_.at(idh)->plan_ways_->target_);
    return std::abs(pointy.x_ - pointidh.x_) + std::abs(pointy.y_ - pointidh.y_) + std::abs(pointy.ele_ - pointidh.ele_);
}

double Globalplan::Calculatef(const int idf) const 
{
    return plan_ways_map_.at(idf)->G_ + plan_ways_map_.at(idf)->H_;
}

int Globalplan::Findleastf(const std::list<int> &listx) const 
{
    if(listx.empty()) return -1;
    int idres = listx.front();
    for(auto it = listx.begin(); it != listx.end(); ++it)
    {
        //std::cout << plan_ways_map[*it].F << std::endl;
        if(plan_ways_map_.at(*it)->F_ < plan_ways_map_.at(idres)->F_)
        {
            idres = *it;
        }
    }
    return idres;
}

std::vector<int> Globalplan::GetNextNode(const int idx) const 
{
    std::vector<int> res;
    for(auto it = plan_ways_map_.at(idx)->next_; it != nullptr; it = it->next_)
    {
        //如果该点不在关闭列表,把它加入待选点列表
        if(!IsinList(it->thisway_->ID_, closelist_))
        {
            res.push_back(it->thisway_->ID_);
        }
    }
    //看当前路段的左右路段是否可行------换道
    if(plan_centerways_->Find(idx)->neighbours_.first != -1)
    {
        if(!IsinList(plan_centerways_->Find(idx)->neighbours_.first, closelist_))
        {
            res.push_back(plan_centerways_->Find(idx)->neighbours_.first);
        }
    }
    if(plan_centerways_->Find(idx)->neighbours_.second != -1)
    {
        if(!IsinList(plan_centerways_->Find(idx)->neighbours_.second, closelist_))
        {
            res.push_back(plan_centerways_->Find(idx)->neighbours_.second);
        }
    }
    return res;
}

bool Globalplan::IsinList(const int x, const std::list<int> &listx) const 
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

int Globalplan::AtWhichPoint(const map::centerway::CenterPoint3D &a, const map::centerway::CenterWay3D *centerline) const
{
    // 1st
    // for(int i = 0; i < centerline_->length - 1; ++i)
    // {
    //     //a点沿道路点集顺序y轴方向向量#1
    //     double oyx = plan_centerways->Findcenterpoint(centerline_->centernodeline[i+1])->x - plan_centerways->Findcenterpoint(centerline_->centernodeline[i])->x;
    //     double oyy = plan_centerways->Findcenterpoint(centerline_->centernodeline[i+1])->y - plan_centerways->Findcenterpoint(centerline_->centernodeline[i])->y;
    //     //a->p0 #2
    //     double ap0x = plan_centerways->Findcenterpoint(centerline_->centernodeline[i])->x - a.x;
    //     double ap0y = plan_centerways->Findcenterpoint(centerline_->centernodeline[i])->y - a.y;
    //     //a->p1 #3
    //     double ap1x = plan_centerways->Findcenterpoint(centerline_->centernodeline[i+1])->x - a.x;
    //     double ap1y = plan_centerways->Findcenterpoint(centerline_->centernodeline[i+1])->y - a.y;
    //     //#1 * #2 与 #1 * #3异号
    //     if((oyx*ap0x + oyy*ap0y) * (oyx*ap1x + oyy*ap1y) < 0) return centerline_->centernodeline[i+1];
    // }

    // 2nd
    for(int i = 0; i < centerline->length_ - 1; ++i)
    {
        double pathx = plan_centerways_->FindCenterPoint(centerline->centernodeline_[i+1])->x_ - plan_centerways_->FindCenterPoint(centerline->centernodeline_[i])->x_;
        double pathy = plan_centerways_->FindCenterPoint(centerline->centernodeline_[i+1])->y_ - plan_centerways_->FindCenterPoint(centerline->centernodeline_[i])->y_;
        double carx = plan_centerways_->FindCenterPoint(centerline->centernodeline_[i+1])->x_ - a.x_;
        double cary = plan_centerways_->FindCenterPoint(centerline->centernodeline_[i+1])->y_ - a.y_;

        if(carx * pathx + cary * pathy > 0) return centerline->centernodeline_[i+1];
    }
    return -1;
}

bool Globalplan::IsIntersect(const map::centerway::CenterPoint3D &a, const map::centerway::CenterPoint3D &b, 
                             const map::node::Point3D &c, const map::node::Point3D &d) const
{
    bool flag = false;

    //a->c
    double acx = c.local_x_ - a.x_;
    double acy = c.local_y_ - a.y_;
    //a->b
    double abx = b.x_ - a.x_;
    double aby = b.y_ - a.y_;
    //a->d
    double adx = d.local_x_ - a.x_;
    double ady = d.local_y_ - a.y_;
    //判别CD在AB两侧, AB X AC 与 AB X AD异号---条件1
    double p1 = abx*acy - aby*acx;
    double p2 = abx*ady - aby*adx;

    //c->a
    double cax = -acx;
    double cay = -acy;
    //c->d
    double cdx = d.local_x_ - c.local_x_;
    double cdy = d.local_y_ - c.local_y_;
    //c->b
    double cbx = b.x_ - c.local_x_;
    double cby = b.y_ - c.local_y_;
    //判别AB在CD两侧, CD X CA 与 CD X CB异号---条件2
    double p3 = cdx*cay - cdy*cax;
    double p4 = cdx*cby - cdy*cbx;

    //判别条件1和条件2同时成立
    if(p1*p2 < 0 && p3*p4 < 0) flag = true;

    return flag;
}

int Globalplan::InWhichCenterway(const map::centerway::CenterPoint3D &a, const map::node::Node *nodes, 
                        const map::way::Way *ways, const map::relation::Relation *relations) const
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

    //2nd, 射线法
    std::vector<map::node::Point3D*> polygon;
    for(auto it = relations->Begin(); it != relations->End(); ++it)
    {
        //std::cout << "in relation: " << it->second->ID << std::endl;
        if(it->second->type_ != map::relation::RelationType::lanelet) continue;
        if(it->second->subtype_ == map::relation::RelationSubType::crosswalk) continue;
        polygon.clear();
        map::way::Line *left = ways->Find(it->second->leftedge_.ID_);
        map::way::Line *right = ways->Find(it->second->rightedge_.ID_);
        double bbox_left = DBL_MAX;
        double bbox_bottom = DBL_MAX;
        double bbox_right = -DBL_MAX;
        double bbox_top = -DBL_MAX;
        for(int i = 0; i < left->length_; ++i)
        {
            map::node::Point3D temp = *nodes->Find(left->nodeline_[i]);
            polygon.push_back(nodes->Find(left->nodeline_[i]));
            if(temp.local_x_ < bbox_left) bbox_left = temp.local_x_;
            if(temp.local_x_ > bbox_right) bbox_right = temp.local_x_;
            if(temp.local_y_ < bbox_bottom) bbox_bottom = temp.local_y_;
            if(temp.local_y_ > bbox_top) bbox_top = temp.local_y_;
        }
        for(int i = right->length_ - 1; i >= 0; --i)
        {
            map::node::Point3D temp = *nodes->Find(right->nodeline_[i]);
            polygon.push_back(nodes->Find(right->nodeline_[i]));
            if(temp.local_x_ < bbox_left) bbox_left = temp.local_x_;
            if(temp.local_x_ > bbox_right) bbox_right = temp.local_x_;
            if(temp.local_y_ < bbox_bottom) bbox_bottom = temp.local_y_;
            if(temp.local_y_ > bbox_top) bbox_top = temp.local_y_;
        }
        if(a.x_ < bbox_left || a.x_ > bbox_right || a.y_ < bbox_bottom || a.y_ > bbox_top) continue;
        //射线，a->b(b与a等高，但超过多边形的最大右边界10米)
        map::centerway::CenterPoint3D b(bbox_right + 10, a.y_);
        //计数穿过多边形次数
        int count = 0;
        //std::cout << it->first << ", polygon size: " << polygon.size() << std::endl;
        for(int i = 0; i < polygon.size(); ++i)
        {
            if(i == polygon.size() - 1)
            {
                if(IsIntersect(a, b, *polygon[i], *polygon[0])) count++;
            }else{
                if(IsIntersect(a, b, *polygon[i], *polygon[i+1])) count++;
            }
        }
        //std::cout << "count: " << count << std::endl;
        if(count % 2 == 1) return it->second->ID_;
    }
    return -1;
}

std::vector<int> Globalplan::LocateLanelets(const map::centerway::CenterPoint3D &a, const map::node::Node *nodes, const map::way::Way *ways, const map::relation::Relation *relations) const
{
    std::vector<int> lanelets;
    std::vector<map::node::Point3D*> polygon;
    for(auto it = relations->Begin(); it != relations->End(); ++it)
    {
        //std::cout << "in relation: " << it->second->ID << std::endl;
        if(it->second->subtype_ == map::relation::RelationSubType::crosswalk) continue;
        polygon.clear();
        if(it->second->type_ != map::relation::RelationType::lanelet) continue;
        map::way::Line *left = ways->Find(it->second->leftedge_.ID_);
        map::way::Line *right = ways->Find(it->second->rightedge_.ID_);
        double bbox_left = DBL_MAX;
        double bbox_bottom = DBL_MAX;
        double bbox_right = -DBL_MAX;
        double bbox_top = -DBL_MAX;
        for(int i = 0; i < left->length_; ++i)
        {
            map::node::Point3D temp = *nodes->Find(left->nodeline_[i]);
            polygon.push_back(nodes->Find(left->nodeline_[i]));
            if(temp.local_x_ < bbox_left) bbox_left = temp.local_x_;
            if(temp.local_x_ > bbox_right) bbox_right = temp.local_x_;
            if(temp.local_y_ < bbox_bottom) bbox_bottom = temp.local_y_;
            if(temp.local_y_ > bbox_top) bbox_top = temp.local_y_;
        }
        for(int i = right->length_ - 1; i >= 0; --i)
        {
            map::node::Point3D temp = *nodes->Find(right->nodeline_[i]);
            polygon.push_back(nodes->Find(right->nodeline_[i]));
            if(temp.local_x_ < bbox_left) bbox_left = temp.local_x_;
            if(temp.local_x_ > bbox_right) bbox_right = temp.local_x_;
            if(temp.local_y_ < bbox_bottom) bbox_bottom = temp.local_y_;
            if(temp.local_y_ > bbox_top) bbox_top = temp.local_y_;
        }
        if(a.x_ < bbox_left || a.x_ > bbox_right || a.y_ < bbox_bottom || a.y_ > bbox_top) continue;
        //射线，a->b(b与a等高，但超过多边形的最大右边界10米)
        map::centerway::CenterPoint3D b(bbox_right + 10, a.y_);
        //计数穿过多边形次数
        int count = 0;
        //std::cout << it->first << ", polygon size: " << polygon.size() << std::endl;
        for(int i = 0; i < polygon.size(); ++i)
        {
            if(i == polygon.size() - 1)
            {
                if(IsIntersect(a, b, *polygon[i], *polygon[0])) count++;
            }else{
                if(IsIntersect(a, b, *polygon[i], *polygon[i+1])) count++;
            }
        }
        //std::cout << "count: " << count << std::endl;
        if(count % 2 == 1) lanelets.push_back(it->second->ID_);
    }
    return lanelets;
}

double Globalplan::Point2EdgeDistance(const map::centerway::CenterPoint3D &a, const map::node::Node *nodes, map::way::Line *line, int pathid) const
{
    //int pathid = Inwhichcenterway(a);
    map::centerway::CenterWay3D *path = plan_centerways_->Find(pathid);
    int i;//a所在的中心线两端点之一
    bool flag = false;
    //定位目标点a,确定a点y方向
    for(i = 0; i < path->length_ - 1; ++i)
    {
        //道路中心线向量#1，i->i+1
        double yy = plan_centerways_->FindCenterPoint(path->centernodeline_[i+1])->y_ - 
                    plan_centerways_->FindCenterPoint(path->centernodeline_[i])->y_;
        double xx = plan_centerways_->FindCenterPoint(path->centernodeline_[i+1])->x_ - 
                    plan_centerways_->FindCenterPoint(path->centernodeline_[i])->x_;
            
        //向量#2，a->i+1
        double c1y = plan_centerways_->FindCenterPoint(path->centernodeline_[i+1])->y_ - a.y_;
        double c1x = plan_centerways_->FindCenterPoint(path->centernodeline_[i+1])->x_ - a.x_;
        //向量#3，a->i
        double c0y = plan_centerways_->FindCenterPoint(path->centernodeline_[i])->y_ - a.y_;
        double c0x = plan_centerways_->FindCenterPoint(path->centernodeline_[i])->x_ - a.x_;
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
    double oy_y = plan_centerways_->FindCenterPoint(path->centernodeline_[i+1])->y_ - 
                plan_centerways_->FindCenterPoint(path->centernodeline_[i])->y_;
    double oy_x = plan_centerways_->FindCenterPoint(path->centernodeline_[i+1])->x_ - 
                plan_centerways_->FindCenterPoint(path->centernodeline_[i])->x_;
    for(j = 0; j < line->length_ - 1; ++j)
    {
        //向量#5
        double p1y = nodes->Find(line->nodeline_[j+1])->local_y_ - a.y_;
        double p1x = nodes->Find(line->nodeline_[j+1])->local_x_ - a.x_;

        //向量#6
        double p0y = nodes->Find(line->nodeline_[j])->local_y_ - a.y_;
        double p0x = nodes->Find(line->nodeline_[j])->local_x_ - a.x_;

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
    double l1y = nodes->Find(line->nodeline_[j+1])->local_y_ - a.y_;
    double l1x = nodes->Find(line->nodeline_[j+1])->local_x_ - a.x_;

    //向量#8
    double l0y = nodes->Find(line->nodeline_[j])->local_y_ - a.y_;
    double l0x = nodes->Find(line->nodeline_[j])->local_x_ - a.x_;

    //#7 x #8 = distance(l1, l0) * 距离
    double d = std::sqrt((l1y-l0y)*(l1y-l0y) + (l1x-l0x)*(l1x-l0x));
    return std::fabs((l1x*l0y - l1y*l0x) / d);
}

std::vector<int> Globalplan::FindNextLanes(const int id) const
{
    std::vector<int> res;
    res.push_back(id);
    auto it = plan_ways_map_.at(id)->next_;
    while(it != nullptr)
    {
        res.push_back(it->thisway_->ID_);
        it = it->next_;
    }
    std::vector<int> resall;
    for(int i = 0; i < res.size(); ++i)
    {
        plan_centerways_->FindNeighbor(res[i], resall);
    }
    return resall;
}

//cost: 从x路段的target到y路段的target长度
void Globalplan::Astar(const int x, const int y)
{
    if(x == y)
    {
        plan_path_.clear();
        plan_path_.push_back(x);
        isfindpath_ = true;
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << "* find! *" << std::endl;
        return;
    }

    openlist_.clear();
    closelist_.clear();
    //放入起点
    openlist_.push_back(x);
    plan_ways_map_[x]->G_ = 0;
    plan_ways_map_[x]->H_ = Calculateh(y, x);
    plan_ways_map_[x]->F_ = Calculatef(x);
    while (!openlist_.empty())
    {
        int minidnode = Findleastf(openlist_);//找到F值最小的点

        openlist_.remove(minidnode);//从开启列表中删除
        closelist_.push_back(minidnode);//放到关闭列表

        //1,找到下一步待选点
        std::vector<int> candiates = GetNextNode(minidnode);
        for(int i = 0; i < candiates.size(); ++i)
        {
            //2,对某一点，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
            if(!IsinList(candiates[i], openlist_))
            {
                plan_ways_map_[candiates[i]]->parent_ = minidnode;
                plan_ways_map_[candiates[i]]->G_ = Calculateg(x, candiates[i]);
                plan_ways_map_[candiates[i]]->H_ = Calculateh(y, candiates[i]);
                plan_ways_map_[candiates[i]]->F_ = Calculatef(candiates[i]);
                openlist_.push_back(candiates[i]);
            }else{
                //3，对某一点，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
                double tempg = Calculateg(x, candiates[i]);
                if(tempg > plan_ways_map_[candiates[i]]->G_)
                {
                    continue;
                }else{
                    plan_ways_map_[candiates[i]]->parent_ = minidnode;
                    plan_ways_map_[candiates[i]]->G_ = tempg;
                    plan_ways_map_[candiates[i]]->F_ = Calculatef(candiates[i]);
                }
            }
            //如果结束点出现在openlist则搜索成功
            if(IsinList(y, openlist_))
            {
                plan_ways_map_[y]->parent_ = minidnode;
                std::cout << "---------------------------------------------------" << std::endl;
                std::cout << "* find! *" << std::endl;
                int i = y;
                while(i != -1)
                {
                    plan_path_.push_back(i);
                    i = plan_ways_map_[i]->parent_;
                }
                std::reverse(plan_path_.begin(), plan_path_.end());
                isfindpath_ = true;
                return;
            }
        }
    }
    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << "* not find! *" << std::endl;
    return;
}

//H恒等于0的A*
void Globalplan::Dijkstra(const int x, const int y)
{
    openlist_.clear();
    closelist_.clear();
    openlist_.push_back(x);
    plan_ways_map_[x]->G_ = 0;
    plan_ways_map_[x]->H_ = 0;
    plan_ways_map_[x]->F_ = Calculatef(x);
    while(!openlist_.empty())
    {
        int minidnode = Findleastf(openlist_);

        openlist_.remove(minidnode);
        closelist_.push_back(minidnode);

        std::vector<int> candiates = GetNextNode(minidnode);
        for(int i = 0; i < candiates.size(); ++i)
        {
            if(!IsinList(candiates[i], openlist_))
            {
                plan_ways_map_[candiates[i]]->parent_ = minidnode;
                plan_ways_map_[candiates[i]]->G_ = Calculateg(x, candiates[i]);
                plan_ways_map_[candiates[i]]->H_ = 0;
                plan_ways_map_[candiates[i]]->F_ = Calculatef(candiates[i]);
                openlist_.push_back(candiates[i]);
            }else{
                double tempg = Calculateg(x, candiates[i]);
                if(tempg > plan_ways_map_[candiates[i]]->G_)
                {
                    continue;
                }else{
                    plan_ways_map_[candiates[i]]->parent_ = minidnode;
                    plan_ways_map_[candiates[i]]->G_ = tempg;
                    plan_ways_map_[candiates[i]]->F_ = Calculatef(candiates[i]);
                }
            }

            if(IsinList(y, openlist_))
            {
                plan_ways_map_[y]->parent_ = minidnode;
                std::cout << "---------------------------------------------------" << std::endl;
                std::cout << "find!" << std::endl;
                int i = y;
                while(i != -1)
                {
                    plan_path_.push_back(i);
                    i = plan_ways_map_[i]->parent_;
                }
                std::reverse(plan_path_.begin(), plan_path_.end());
                isfindpath_ = true;
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
    std::cout << "* replan ... *" << std::endl;
    openlist_.clear();
    closelist_.clear();
    plan_path_.clear();
    isfindpath_ = false;
    for(auto it = plan_ways_map_.begin(); it != plan_ways_map_.end(); ++it)
    {
        it->second->F_ = DBL_MAX;
        it->second->G_ = DBL_MAX;
        it->second->H_ = DBL_MAX;
        it->second->parent_ = -1;
    }
}

bool Globalplan::IsReset(const int x, const int y)
{
    if(plan_path_.empty()) return true;
    
    bool flag = true;
    int i = y;
    std::vector<int> plan_path_new;
    while(i != -1)
    {
        if(i == x)
        {
            plan_path_new.push_back(i);
            std::reverse(plan_path_new.begin(), plan_path_new.end());
            plan_path_.clear();
            plan_path_ = plan_path_new;
            flag = false;
            std::cout << "* not necessary to replan! *" << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;
            break;
        }
        plan_path_new.push_back(i);
        i = plan_ways_map_[i]->parent_;
    }
    return flag;
}

std::vector<int> Globalplan::Run(const int x, const int y)
{
    //test
    /*map::centerway::CenterPoint3D testa(6.4, 1.2);//6.4, 1.2->220
    int testares = Inwhichcenterway(testa);
    std::cout << "testa is in " << testares << " path" << std::endl;*/

    //replan
    if(IsReset(x, y))
    {
        Reset();
        Astar(x, y);
    }

    //feedback
    if(isfindpath_)
    {
        std::cout << "start point in path: " << plan_path_[0] << 
                     ", goal point in path: " << plan_path_[plan_path_.size()-1] << std::endl;
        std::cout << "the path as follow: ";
        for(int i = 0; i < plan_path_.size(); i++)
        {
            if(i != plan_path_.size() - 1)
            {
                //std::cout << plan_path[i] << " " << plan_ways_map[plan_path[i]]->F << " ---> " ;
                std::cout << plan_path_[i] << " ---> " ;
            }else{
                std::cout << plan_path_[i];
            }
        }
        std::cout << std::endl;
        //std::cout << "cost: " << plan_ways_map[*(--plan_path.end())]->F << std::endl;
    }
    return plan_path_;
}

void Globalplan::DeleteChain(plan_ways_map_chain *root)
{
    if(root == nullptr) return;
    DeleteChain(root->next_);
    delete root;
}

Globalplan::~Globalplan()
{
    // std::cout << "~Globalplan" << std::endl;
    for(auto it = plan_ways_map_.begin(); it != plan_ways_map_.end(); ++it)
    {
        DeleteChain(it->second->next_);
        delete it->second;
    }
    plan_ways_map_.clear();
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
 
	temp[0].x_ = 2*temp[0].x_ - temp[1].x_;// 将折线两端点换成延长线上两点
	temp[0].y_ = 2*temp[0].y_ - temp[1].y_;
    temp[0].ele_ = 2*temp[0].ele_ - temp[1].ele_;
 
	temp[Num-1].x_ = 2*temp[Num-1].x_ - temp[Num-2].x_;
	temp[Num-1].y_ = 2*temp[Num-1].y_ - temp[Num-2].y_;
    temp[Num-1].ele_ = 2*temp[Num-1].ele_ - temp[Num-2].ele_;
 
	map::centerway::CenterPoint3D NodePt1, NodePt2, NodePt3;
	double t;
    for(int i = 0; i < Num - 2; i++)
	{
		NodePt1 = temp[i]; 
        NodePt2 = temp[i+1]; 
        NodePt3 = temp[i+2];
		if(i == 0){ // 第一段取t=0和t=0.5点   
			t = 0;
			pt[i].x_ = F02(t)*NodePt1.x_ + F12(t)*NodePt2.x_ + F22(t)*NodePt3.x_;
			pt[i].y_ = F02(t)*NodePt1.y_ + F12(t)*NodePt2.y_ + F22(t)*NodePt3.y_;
            pt[i].ele_ = F02(t)*NodePt1.ele_ + F12(t)*NodePt2.ele_ + F22(t)*NodePt3.ele_;
			t = 0.5;
			pt[i+1].x_ = F02(t)*NodePt1.x_ + F12(t)*NodePt2.x_ + F22(t)*NodePt3.x_;
			pt[i+1].y_ = F02(t)*NodePt1.y_ + F12(t)*NodePt2.y_ + F22(t)*NodePt3.y_;
            pt[i+1].ele_ = F02(t)*NodePt1.ele_ + F12(t)*NodePt2.ele_ + F22(t)*NodePt3.ele_;
		}else if(i == Num - 3){ // 最后一段取t=0.5和t=1点
			t = 0.5;
			pt[i+1].x_ = F02(t)*NodePt1.x_ + F12(t)*NodePt2.x_ + F22(t)*NodePt3.x_;
			pt[i+1].y_ = F02(t)*NodePt1.y_ + F12(t)*NodePt2.y_ + F22(t)*NodePt3.y_;
            pt[i+1].ele_ = F02(t)*NodePt1.ele_ + F12(t)*NodePt2.ele_ + F22(t)*NodePt3.ele_;
			t = 1;
			pt[i+2].x_ = F02(t)*NodePt1.x_ + F12(t)*NodePt2.x_ + F22(t)*NodePt3.x_;
			pt[i+2].y_ = F02(t)*NodePt1.y_ + F12(t)*NodePt2.y_ + F22(t)*NodePt3.y_;
            pt[i+2].ele_ = F02(t)*NodePt1.ele_ + F12(t)*NodePt2.ele_ + F22(t)*NodePt3.ele_;
		}else{ // 中间段取t=0.5点
			t = 0.5;
			pt[i+1].x_ = F02(t)*NodePt1.x_ + F12(t)*NodePt2.x_ + F22(t)*NodePt3.x_;
			pt[i+1].y_ = F02(t)*NodePt1.y_ + F12(t)*NodePt2.y_ + F22(t)*NodePt3.y_;
            pt[i+1].ele_ = F02(t)*NodePt1.ele_ + F12(t)*NodePt2.ele_ + F22(t)*NodePt3.ele_;
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
 
	temp[0].x_ = 2*temp[0].x_ - temp[1].x_; // 将折线两端点换成延长线上两点
	temp[0].y_ = 2*temp[0].y_ - temp[1].y_;
    temp[0].ele_ = 2*temp[0].ele_ - temp[1].ele_;
 
	temp[Num-1].x_ = 2*temp[Num-1].x_ - temp[Num-2].x_;
	temp[Num-1].y_ = 2*temp[Num-1].y_ - temp[Num-2].y_;
    temp[Num-1].ele_ = 2*temp[Num-1].ele_ - temp[Num-2].ele_;
 
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
				pt[totalnum].x_ = F02(t)*NodePt2.x_ + F12(t)*NodePt3.x_ + F22(t)*NodePt4.x_;
				pt[totalnum].y_ = F02(t)*NodePt2.y_ + F12(t)*NodePt3.y_ + F22(t)*NodePt4.y_;
                pt[totalnum].ele_ = F02(t)*NodePt2.ele_ + F12(t)*NodePt3.ele_ + F22(t)*NodePt4.ele_;
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
				pt[totalnum].x_ = F02(t)*NodePt1.x_ + F12(t)*NodePt2.x_ + F22(t)*NodePt3.x_;
				pt[totalnum].y_ = F02(t)*NodePt1.y_ + F12(t)*NodePt2.y_ + F22(t)*NodePt3.y_;
                pt[totalnum].ele_ = F02(t)*NodePt1.ele_ + F12(t)*NodePt2.ele_ + F22(t)*NodePt3.ele_;
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
				pt[totalnum].x_ = F02(t)*NodePt1.x_ + F12(t)*NodePt2.x_ + F22(t)*NodePt3.x_;
				pt[totalnum].y_ = F02(t)*NodePt1.y_ + F12(t)*NodePt2.y_ + F22(t)*NodePt3.y_;
                pt[totalnum].ele_ = F02(t)*NodePt1.ele_ + F12(t)*NodePt2.ele_ + F22(t)*NodePt3.ele_;
				totalnum++;
			}
 
			for(int j = 0; j < RightInsertNum; j++)
			{				
				t = rightoffset + Rightdt*j;
				pt[totalnum].x_ = F02(t)*NodePt2.x_ + F12(t)*NodePt3.x_ + F22(t)*NodePt4.x_;
				pt[totalnum].y_ = F02(t)*NodePt2.y_ + F12(t)*NodePt3.y_ + F22(t)*NodePt4.y_;
                pt[totalnum].ele_ = F02(t)*NodePt2.ele_ + F12(t)*NodePt3.ele_ + F22(t)*NodePt4.ele_;
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
 
	temp[0].x_ = 2*temp[1].x_ - temp[2].x_; // 将折线延长线上两点加入作为首点和尾点
	temp[0].y_ = 2*temp[1].y_ - temp[2].y_;
    temp[0].ele_ = 2*temp[1].ele_ - temp[2].ele_;
 
	temp[Num+1].x_ = 2*temp[Num].x_ - temp[Num-1].x_;
	temp[Num+1].y_ = 2*temp[Num].y_ - temp[Num-1].y_;
    temp[Num+1].ele_ = 2*temp[Num].ele_ - temp[Num-1].ele_;
 
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
			pt[i].x_ = F03(t)*NodePt1.x_ + F13(t)*NodePt2.x_ + F23(t)*NodePt3.x_ + F33(t)*NodePt4.x_;
			pt[i].y_ = F03(t)*NodePt1.y_ + F13(t)*NodePt2.y_ + F23(t)*NodePt3.y_ + F33(t)*NodePt4.y_;
            pt[i].ele_ = F03(t)*NodePt1.ele_ + F13(t)*NodePt2.ele_ + F23(t)*NodePt3.ele_ + F33(t)*NodePt4.ele_;
			t = 1;
			pt[i+1].x_ = F03(t)*NodePt1.x_ + F13(t)*NodePt2.x_ + F23(t)*NodePt3.x_ + F33(t)*NodePt4.x_;
			pt[i+1].y_ = F03(t)*NodePt1.y_ + F13(t)*NodePt2.y_ + F23(t)*NodePt3.y_ + F33(t)*NodePt4.y_;
            pt[i+1].ele_ = F03(t)*NodePt1.ele_ + F13(t)*NodePt2.ele_ + F23(t)*NodePt3.ele_ + F33(t)*NodePt4.ele_;
		}else{ // 中间段取t=0.5点
			t = 0;
			pt[i].x_ = F03(t)*NodePt1.x_ + F13(t)*NodePt2.x_ + F23(t)*NodePt3.x_ + F33(t)*NodePt4.x_;
			pt[i].y_ = F03(t)*NodePt1.y_ + F13(t)*NodePt2.y_ + F23(t)*NodePt3.y_ + F33(t)*NodePt4.y_;
            pt[i].ele_ = F03(t)*NodePt1.ele_ + F13(t)*NodePt2.ele_ + F23(t)*NodePt3.ele_ + F33(t)*NodePt4.ele_;
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
 
	temp[0].x_ = 2*temp[1].x_ - temp[2].x_;// 将折线延长线上两点加入作为首点和尾点
	temp[0].y_ = 2*temp[1].y_ - temp[2].y_;
    temp[0].ele_ = 2*temp[1].ele_ - temp[2].ele_;
 
	temp[Num+1].x_ = 2*temp[Num].x_ - temp[Num-1].x_;
	temp[Num+1].y_ = 2*temp[Num].y_ - temp[Num-1].y_;
    temp[Num+1].ele_ = 2*temp[Num].ele_ - temp[Num-1].ele_;
 
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
			pt[totalnum].x_ = F03(t)*NodePt1.x_ + F13(t)*NodePt2.x_ + F23(t)*NodePt3.x_ + F33(t)*NodePt4.x_;
			pt[totalnum].y_ = F03(t)*NodePt1.y_ + F13(t)*NodePt2.y_ + F23(t)*NodePt3.y_ + F33(t)*NodePt4.y_;
            pt[totalnum].ele_ = F03(t)*NodePt1.ele_ + F13(t)*NodePt2.ele_ + F23(t)*NodePt3.ele_ + F33(t)*NodePt4.ele_;
			totalnum++;
		}
 
		if(i == Num - 2){ // 最后一个尾点
			t = 1;
			pt[totalnum].x_ = F03(t)*NodePt1.x_ + F13(t)*NodePt2.x_ + F23(t)*NodePt3.x_ + F33(t)*NodePt4.x_;
			pt[totalnum].y_ = F03(t)*NodePt1.y_ + F13(t)*NodePt2.y_ + F23(t)*NodePt3.y_ + F33(t)*NodePt4.y_;
            pt[totalnum].ele_ = F03(t)*NodePt1.ele_ + F13(t)*NodePt2.ele_ + F23(t)*NodePt3.ele_ + F33(t)*NodePt4.ele_;
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




