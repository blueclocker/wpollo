/*
 * @Author: your name
 * @Date: 2022-03-13 15:21:33
 * @LastEditTime: 2022-03-20 22:38:11
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/map_plan.cpp
 */
#include "../include/osmmap/map_plan.h"

namespace plan
{

Globalplan::Globalplan(map::centerway::CenterWay *plan_centerways_):plan_centerways(plan_centerways_)
{
    for(auto it = plan_centerways->Begin(); it != plan_centerways->End(); ++it)
    {
        Planmap *aplan_way = new Planmap(it->second);
        //plan_ways_map.insert(it->second->ID, aplan_way);
        plan_ways_map[it->second->ID] = aplan_way;
    }
    CreatePlanmap();
}

double Globalplan::Centerpoint3d_distance(map::centerway::CenterPoint3D *a, map::centerway::CenterPoint3D *b)
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

int Globalplan::Inwhichcenterway(map::centerway::CenterPoint3D a)
{
    for(auto it = plan_centerways->Begin(); it != plan_centerways->End(); ++it)
    {
        double atosource = Centerpoint3d_distance(&a, plan_centerways->Findcenterpoint(it->second->source));
        double atotarget = Centerpoint3d_distance(&a, plan_centerways->Findcenterpoint(it->second->target));
        double sourcetotarget = Centerwaylength(it->second);
        if((atosource + atotarget - sourcetotarget) < sourcetotarget * 0.05)
        {
            return it->second->ID;
        }
    }
    return -1;
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
    std::cout << "---------------------------------------------------" << std::endl;
    return;
}

void Globalplan::Reset()
{
    openlist.clear();
    closelist.clear();
    plan_path.clear();
    for(auto it = plan_ways_map.begin(); it != plan_ways_map.end(); ++it)
    {
        it->second->F = DBL_MAX;
        it->second->G = DBL_MAX;
        it->second->H = DBL_MAX;
        it->second->parent = -1;
    }
}

std::vector<int> Globalplan::run(int x, int y)
{
    //test
    /*map::centerway::CenterPoint3D testa(6.4, 1.2);//6.4, 1.2->220
    int testares = Inwhichcenterway(testa);
    std::cout << "testa is in " << testares << " path" << std::endl;*/

    Reset();
    Astar(x, y);
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
        std::cout << "cost: " << plan_ways_map[*(--plan_path.end())]->F << std::endl;
        std::cout << "---------------------------------------------------" << std::endl;
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





};//namespace plan




