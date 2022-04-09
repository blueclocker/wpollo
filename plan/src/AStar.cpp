/*
 * @Author: your name
 * @Date: 2021-11-22 13:13:44
 * @LastEditTime: 2022-03-31 22:16:53
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/src/AStar.cpp
 */
#include "plan/AStar.h"


AStar::AStar(Node* start, Node* end, const std::vector<std::vector<int>> &map_, const bool isIgnoreCorner_)
{
    isIgnoreCorner = isIgnoreCorner_;
    if(init(start, end, map_))
    {
        std::cout << "init successful!" << std::endl;
    }else{
        std::cout << "init fail!" << std::endl;
    }
}

AStar::AStar()
{
}

AStar::~AStar()
{
}

bool AStar::init(Node* start_, Node* end_, const std::vector<std::vector<int>> &map_)
{
    if(start_ == nullptr || end_ == nullptr) return false;
    start_node = start_;
    end_node = end_;
    map = map_;
    //closelist.clear();
    //openlist.clear();
    return true;
}

int AStar::calcG(const Node* last_node, const Node* current_node)
{
    int incleaseg = (std::abs(current_node->x - last_node->x) + std::abs(current_node->y - last_node->y)) == 1 ? wightw : wighth;
    int parentg = current_node->father == nullptr ? 0 : current_node->father->g;
    return incleaseg + parentg;
}

int AStar::calcH(const Node* current_node)
{
    double tempx = 1.0*(end_node->x - current_node->x);
    double tempy = 1.0*(end_node->y - current_node->y);
    return std::sqrt(tempx * tempx + tempy * tempy) * wightw;
}

int AStar::calcF(const Node* current_node)
{
    return current_node->g + current_node->h;
}

Node* AStar::getLeastFnode()
{
    if(!openlist.empty())
    {
        Node* resnode = openlist.front();
        for(auto iter = openlist.begin(); iter != openlist.end(); ++iter)
        {
            if((*iter)->f < resnode->f)
            {
                resnode = *iter;
            }
        }
        return resnode;
    }
    return nullptr;
}

bool AStar::isCanReach(const Node* current_node, const Node* next_node)
{
    if(next_node->x < 0 || next_node->x > map.size()-1//x越界
     ||next_node->y < 0 || next_node->y > map[0].size()-1//y越界
     ||map[next_node->x][next_node->y] == 1//下一点是障碍物 map[next_node->x][next_node->y] == 1
     ||(next_node->x == current_node->x && next_node->y == current_node->y)//排除自身点
     ||isInCloselist(next_node))//下一点已经访问完成
    {
        return false;
    }else{
        if(std::abs(next_node->x-current_node->x)+std::abs(next_node->y-current_node->y) == 1)//下一点走直角
        {
            return true;
        }else{//下一点走斜角
            if(map[current_node->x][next_node->y] == 0 && map[next_node->x][current_node->y] == 0)
            {
                return true;//斜角的交叉对角可行
            }else{
                return isIgnoreCorner;//斜角交叉对角默认不可行
            }
        }
    }
}

bool AStar::isInCloselist(const Node* current_node)
{
    if(closelist.empty()) return false;
    for(auto iter = closelist.begin(); iter != closelist.end(); ++iter)
    {
        if((*iter)->x == current_node->x && (*iter)->y == current_node->y)
            return true;
    }
    return false;
}

bool AStar::isInOpenlist(const Node* current_node)
{
    if(openlist.empty()) return false;
    for(auto iter = openlist.begin(); iter != openlist.end(); ++iter)
    {
        if((*iter)->x == current_node->x && (*iter)->y == current_node->y)
        {
            return true;
        }
    }
    return false;
}

std::vector<Node*> AStar::getNextNode(const Node* current_node)
{
    std::vector<Node*> nextnodes;
    for(int i = (current_node->x) - 1; i <= (current_node->x) + 1; ++i)
    {
        for(int j = (current_node->y) - 1; j <= (current_node->y) + 1; ++j)
        {
            Node* temp = new Node(i, j);
            if(isCanReach(current_node, temp))
            {
                nextnodes.push_back(temp);
            }else{
                delete temp;
                temp = nullptr;
            }
        }
    }
    //std::cout << "nextnodes" <<std::endl;
    return nextnodes;
}

bool AStar::search()
{
    openlist.push_back(start_node);//输入起点
    //std::cout << "start_node: " << start_node->x << " " << start_node->y << std::endl;
    while(!openlist.empty())
    {
        Node* curnode= getLeastFnode();//找到F值最小的点
        
        openlist.remove(curnode);//从开启列表中删除
        closelist.push_back(curnode); //放到关闭列表
        //1,找到当前周围八个格中可以通过的格子
        std::vector<Node*> condidates = getNextNode(curnode);
        //std::cout << "father search" <<std::endl;
        for(int i = 0; i < condidates.size(); ++i)
        {
            //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
            if(!isInOpenlist(condidates[i]))
            {
                condidates[i]->father = curnode;
                condidates[i]->g = calcG(curnode, condidates[i]);
                condidates[i]->h = calcH(condidates[i]);
                condidates[i]->f = calcF(condidates[i]);
                openlist.push_back(condidates[i]);
            }else{
                //3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
                int tempg = calcG(curnode, condidates[i]);
                if(tempg > condidates[i]->g)
                {
                    continue;
                }else{
                    condidates[i]->father = curnode;
                    condidates[i]->g = tempg;
                    condidates[i]->f = calcF(condidates[i]);
                }
                delete condidates[i];
                condidates[i] = nullptr;
            }
            //如果结束点出现在openlist则搜索成功
            if(isInOpenlist(end_node)) 
            {
                end_node->father = curnode;
                return true;
            }
        }
    }
    std::cout << "Not Find" << std::endl;
    return false;
}

std::list<Node*> AStar::solve()
{
    std::list<Node*> path;
    Node* pathnode = end_node;
    if(search())
    {
        while(pathnode != nullptr)
        {
            path.push_front(pathnode);
            pathnode = pathnode->father;
            //std::cout << "father solve" <<std::endl;
        }
    }
    /*for(int i = 0; i < map.size(); ++i)
    {
        for(int j = 0; j < map[0].size(); ++j)
        {
            std::cout << map[i][j] << " ";
        }
        std::cout << std::endl;
    }*/
    std::cout << "openlist size: " << openlist.size() << std::endl;
    std::cout << "closelist size: " << closelist.size() << std::endl;
    openlist.clear();
    closelist.clear();

    return path;
}

