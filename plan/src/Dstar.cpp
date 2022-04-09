/*
 * @Author: your name
 * @Date: 2022-03-31 15:46:57
 * @LastEditTime: 2022-04-07 22:11:01
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/plan/src/Dstar.cpp
 */
#include "../include/plan/DStar.h"

DStar::DStar(std::vector<std::vector<int>> map_):map(map_)
{
    Dstarmap.resize(map.size());
    for(int i = 0; i < Dstarmap.size(); ++i)
    {
        Dstarmap[i].resize(map[0].size());
    }
    //init Dstarmap
    for(int i = 0; i < Dstarmap.size(); ++i)
    {
        for(int j = 0; j < Dstarmap[i].size(); ++j)
        {
            Dstarmap[i][j] = DNode(i, j);
        }
    }
    std::cout << "dstar init successful!" << std::endl;
}

void DStar::sortOpenlist()
{
    for(int i = 0; i < openlist.size() - 1; ++i)
    {
        for(int j = 0; j < openlist.size()-i-1; ++j)
        {
            if(openlist[j].K > openlist[j+1].K)
            {
                DNode temp = openlist[j];
                openlist[j] = openlist[j+1];
                openlist[j+1] = temp;
            }
        }
    }
}

int DStar::cost(const DNode &a, const DNode &b) const
{
    if(map[a.x][a.y] == 1 || map[b.x][b.y] == 1)
    {
        return INT16_MAX;
    }else{
        return (std::abs(b.x - a.x) + std::abs(b.y - a.y)) == 1 ? wightw : wighth;
    }
}

void DStar::insert(const DNode &a, int h_)
{
    if(!isInList(a, openlist) && !isInList(a, closelist))//if t(x) == new
    {
        Dstarmap[a.x][a.y].K = h_;
    }
    else if(isInList(a, openlist)) //if t(x) == open
    {
        Dstarmap[a.x][a.y].K = std::min(Dstarmap[a.x][a.y].K, h_);
    }
    else if(isInList(a, closelist)) //if t(x) == close
    {
        Dstarmap[a.x][a.y].K = std::min(Dstarmap[a.x][a.y].K, h_);
        //t(x) = open
        openlist.push_back(Dstarmap[a.x][a.y]);
        //delete(x) in closelist
        deleteInList(a, closelist);
    }
    //h(x) = h_;
    //Dstarmap[a.x][a.y].H = h_;
    //sort openlist
    //openlist.push_back(a);
    sortOpenlist();
}

int DStar::calculateH(const DNode &a) const
{
    //if(a.nextx == -1 && a.nexty == -1)
    if(Dstarmap[a.x][a.y].nextx == -1 && Dstarmap[a.x][a.y].nexty == -1)
    {
        return 0;
    }else{
        int incleaseg = (std::abs(Dstarmap[a.nextx][a.nexty].x - a.x) + std::abs(Dstarmap[a.nextx][a.nexty].y - a.y)) == 1 ? wightw : wighth;
        return incleaseg + Dstarmap[a.nextx][a.nexty].H;
    }
}

bool DStar::isInList(const DNode &a, const std::vector<DNode> &list_) const
{
    bool flag = false;
    for(auto it = list_.begin(); it != list_.end(); ++it)
    {
        if((it->x == a.x) && (it->y == a.y))
        {
            flag = true;
            break;
        }
    }
    return flag;
}

void DStar::deleteInList(const DNode &a, std::vector<DNode> &list_)
{
    for(auto it = list_.begin(); it != list_.end(); ++it)
    {
        if(it->x == a.x && it->y == a.y)
        {
            list_.erase(it);
            break;
        }
    }
}

bool DStar::isCanReach(const DNode &current_node, const DNode &next_node) const
{
    if(next_node.x < 0 || next_node.x > map.size()-1//x越界
     ||next_node.y < 0 || next_node.y > map[0].size()-1//y越界
     ||map[next_node.x][next_node.y] == 1//下一点是障碍物 map[next_node->x][next_node->y] == 1
     ||(next_node.x == current_node.x && next_node.y == current_node.y)//排除自身点
     ||isInList(next_node, closelist))//下一点已经访问完成
    {
        return false;
    }else{
        if(std::abs(next_node.x-current_node.x)+std::abs(next_node.y-current_node.y) == 1)//下一点走直角
        {
            return true;
        }else{//下一点走斜角
            if(map[current_node.x][next_node.y] == 0 && map[next_node.x][current_node.y] == 0)
            {
                return true;//斜角的交叉对角可行
            }else{
                return false;//斜角交叉对角默认不可行
            }
        }
    }
}

std::vector<DNode> DStar::getNextNode(const DNode &a) const
{
    std::vector<DNode> nextnodes;
    for(int i = a.x - 1; i <= a.x + 1; ++i)
    {
        for(int j = a.y - 1; j <= a.y + 1; ++j)
        {
            DNode temp = DNode(i, j);
            if(isCanReach(a, temp))
            {
                nextnodes.push_back(temp);
            }else{
                continue;
            }
        }
    }
    //std::cout << "nextnodes" <<std::endl;
    return nextnodes;
}

std::vector<DNode> DStar::getNextNodeReplan(const DNode &a) const
{
    std::vector<DNode> nextnodes;
    for(int i = a.x - 1; i <= a.x + 1; ++i)
    {
        for(int j = a.y - 1; j <= a.y + 1; ++j)
        {
            DNode temp = DNode(i, j);
            if((i == a.x && j == a.y) || i < 0 || i > map.size()-1 || j < 0 || j > map[0].size()-1)
            {
                continue;
            }else{
                if(map[i][j] != 1)
                {
                    nextnodes.push_back(temp);
                }
            }
        }
    }
    //std::cout << "nextnodes" <<std::endl;
    return nextnodes;
}

//一轮扩散
int DStar::processState()
{
    if(openlist.empty()) return -1;
    //从OPENLIST表中取k值最小的节点temp,并在OPENLIST删除, 此时的temp实际是变化点
    DNode temp = *openlist.begin();
    int k_old = temp.K;
    openlist.erase(openlist.begin());
    std::cout << "process in (" << temp.x << ", " << temp.y << "): " <<  k_old << ", " << Dstarmap[temp.x][temp.y].H << std::endl;

    //if k old < h(X)
    if(k_old < Dstarmap[temp.x][temp.y].H)
    {
        //for each neighbor Y of X:
        std::vector<DNode> candiates = getNextNodeReplan(temp);
        for(int i = 0; i < candiates.size(); ++i)
        {
            if(//(isInList(candiates[i], openlist) || isInList(candiates[i], closelist)) //if t(Y) ≠ new
                Dstarmap[candiates[i].x][candiates[i].y].H <= k_old //h(Y) <= kold
             && Dstarmap[temp.x][temp.y].H > Dstarmap[candiates[i].x][candiates[i].y].H + cost(temp, candiates[i])) //h(X) > h(Y) +c(Y,X)
            {
                //b(X) = Y; 
                Dstarmap[temp.x][temp.y].nextx = candiates[i].x;
                Dstarmap[temp.x][temp.y].nexty = candiates[i].y;
                //h(X) = h(Y)+c(Y,X);
                Dstarmap[temp.x][temp.y].H = Dstarmap[candiates[i].x][candiates[i].y].H + cost(temp, candiates[i]);
                //std::cout << "kold <" << std::endl;
            }
        }
    }
    //if k old= h(X) 
    else if(k_old == Dstarmap[temp.x][temp.y].H)
    {
        //for each neighbor Y of X:
        std::vector<DNode> candiates = getNextNodeReplan(temp);
        for(int i = 0; i < candiates.size(); ++i)
        {
            if((!isInList(candiates[i], openlist) && !isInList(candiates[i], closelist)) //if t(Y) == new
              //b(Y) =X and h(Y) ≠ h(X)+c (X,Y)
             || ((Dstarmap[candiates[i].x][candiates[i].y].nextx == temp.x && Dstarmap[candiates[i].x][candiates[i].y].nexty == temp.y)
               &&(Dstarmap[candiates[i].x][candiates[i].y].H != Dstarmap[temp.x][temp.y].H + cost(candiates[i], temp)))
              //b(Y) ≠ X and h(Y) > h(X)+c (X,Y) 
             || ((Dstarmap[candiates[i].x][candiates[i].y].nextx != temp.x && Dstarmap[candiates[i].x][candiates[i].y].nexty != temp.y)
               &&(Dstarmap[candiates[i].x][candiates[i].y].H > Dstarmap[temp.x][temp.y].H + cost(candiates[i], temp))))
            {
                //b(Y) = X ; 
                Dstarmap[candiates[i].x][candiates[i].y].nextx = temp.x;
                Dstarmap[candiates[i].x][candiates[i].y].nexty = temp.y;
                //INSERT(Y, h(X)+c(X,Y))
                insert(candiates[i], Dstarmap[temp.x][temp.y].H + cost(temp, candiates[i]));
                //std::cout << "kold =" << std::endl;
            }
        }
    }
    //if k old> h(X) 
    else
    {
        //for each neighbor Y of X:
        std::vector<DNode> candiates = getNextNodeReplan(temp);
        for(int i = 0; i < candiates.size(); ++i)
        {
            if((!isInList(candiates[i], openlist) && !isInList(candiates[i], closelist))//if t(Y) = NEW
             //b(Y) =X and h(Y) ≠ h(X)+c (X,Y) 
             || ((Dstarmap[candiates[i].x][candiates[i].y].nextx == temp.x && Dstarmap[candiates[i].x][candiates[i].y].nexty == temp.y)
               &&(Dstarmap[candiates[i].x][candiates[i].y].H != Dstarmap[temp.x][temp.y].H + cost(candiates[i], temp))))
            {
                //b(Y) = X ; 
                Dstarmap[candiates[i].x][candiates[i].y].nextx = temp.x;
                Dstarmap[candiates[i].x][candiates[i].y].nexty = temp.y;
                //INSERT(Y, h(X)+c(X,Y))
                insert(candiates[i], Dstarmap[temp.x][temp.y].H + cost(temp, candiates[i]));
                //std::cout << "kold >" << std::endl;
            }
            else
            {
                //b(Y) ≠ X and h(Y) > h(X)+c (X,Y) 
                if(((Dstarmap[candiates[i].x][candiates[i].y].nextx != temp.x && Dstarmap[candiates[i].x][candiates[i].y].nexty != temp.y)
                  &&(Dstarmap[candiates[i].x][candiates[i].y].H > Dstarmap[temp.x][temp.y].H + cost(candiates[i], temp))))
                {
                    //INSERT(X, h(X))
                    insert(temp, Dstarmap[temp.x][temp.y].H);
                }
                else
                {
                    //b(Y) ≠ X and h(X) > h(Y)+c (X,Y)
                    if(((Dstarmap[candiates[i].x][candiates[i].y].nextx != temp.x && Dstarmap[candiates[i].x][candiates[i].y].nexty != temp.y)
                      &&(Dstarmap[temp.x][temp.y].H > Dstarmap[candiates[i].x][candiates[i].y].H + cost(candiates[i], temp)))
                     //t(Y) = CLOSED and h(Y) > kold
                     && (isInList(candiates[i], closelist)) && (Dstarmap[candiates[i].x][candiates[i].y].H > k_old))
                    {
                        //INSERT(Y, h(Y))
                        insert(candiates[i], Dstarmap[candiates[i].x][candiates[i].y].H);
                    }
                }
            }
        }
    }
    //Return GET-KMIN()
    return openlist.begin()->K;

}

bool DStar::Dijstra()
{
    goal.H = 0;
    goal.K = 0;
    Dstarmap[goal.x][goal.y].H = 0;
    Dstarmap[goal.x][goal.y].K = 0;
    openlist.push_back(goal);
    while(!openlist.empty())
    {
        DNode curnode = *openlist.begin();
        openlist.erase(openlist.begin());
        closelist.push_back(curnode);

        //1,找到下一步待选点
        std::vector<DNode> candiates = getNextNode(curnode);
        for(int i = 0; i < candiates.size(); ++i)
        {
            //2,对某一点，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算H
            if(!isInList(candiates[i], openlist))
            {
                Dstarmap[candiates[i].x][candiates[i].y].nextx = curnode.x;
                Dstarmap[candiates[i].x][candiates[i].y].nexty = curnode.y;
                Dstarmap[candiates[i].x][candiates[i].y].H = calculateH(Dstarmap[candiates[i].x][candiates[i].y]);
                //std::cout << "(" << candiates[i].x << ", " << candiates[i].y << "): " << Dstarmap[candiates[i].x][candiates[i].y].H << std::endl;
                Dstarmap[candiates[i].x][candiates[i].y].K = Dstarmap[candiates[i].x][candiates[i].y].H;
                // Dstarmap[candiates[i].x][candiates[i].y] = candiates[i];
                openlist.push_back(Dstarmap[candiates[i].x][candiates[i].y]);
                sortOpenlist();
            }else{
                //3，对某一点，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新H
                int temp = calculateH(Dstarmap[candiates[i].x][candiates[i].y]);
                if(temp < Dstarmap[candiates[i].x][candiates[i].y].H )
                {
                    Dstarmap[candiates[i].x][candiates[i].y].nextx = curnode.x;
                    Dstarmap[candiates[i].x][candiates[i].y].nexty = curnode.y;
                    Dstarmap[candiates[i].x][candiates[i].y].H = temp;
                    //std::cout << "(" << candiates[i].x << ",,, " << candiates[i].y << "): " << temp << std::endl;
                    Dstarmap[candiates[i].x][candiates[i].y].K = temp;
                    // Dstarmap[candiates[i].x][candiates[i].y] = candiates[i];
                }
            }
            //如果开始点出现在openlist则搜索成功
            if(isInList(start, closelist))
            {
                //Dstarmap[start.x][start.y].nextx = curnode.x;
                //Dstarmap[start.x][start.y].nexty = curnode.y;
                std::cout << "dstar find" << std::endl;
                return true;
            }
        }
    }
    std::cout << "dstar not find" << std::endl;
    return false;
}

void DStar::changeMap(int x_, int y_)
{
    map[x_][y_] = 1;
    Dstarmap[x_][y_].H = INT16_MAX;//防止越界
    //openlist.push_back(Dstarmap[x_][y_]);
    //sortOpenlist();
    //sortOpenlist();
    // if(isInList(Dstarmap[x_][y_], closelist))
    // {
    //     //delete(x) in closelist
    //     deleteInList(Dstarmap[x_][y_], closelist);
    // }
}

std::vector<DNode> DStar::rePlan()
{
    std::vector<DNode> res;
    //h(G) = 0;
    Dstarmap[goal.x][goal.y].H = 0;
    int kmin;
    do
    {
        //k_min = PROCESS_STATE(); //主要是减少cost，寻找最优路径
        kmin = processState();
    } while (kmin != -1 && isInList(start, openlist));//while(k_min != -1 && S_not_removed_from_open_list)
    if(kmin == -1)
    {
        std::cout << "DStar not find" << std::endl;//G_unreachable;
        return res;//exit;
    }else{
        DNode temp = Dstarmap[start.x][start.y];
        do
        {
            do
            {
                //trace_optimal_path(); //沿着最优路径行驶{S, ……, G}，根据具体情形来实现
                res.push_back(temp);
                temp = Dstarmap[temp.nextx][temp.nexty];
                //while(G_is_not_reached && map == environment)
            }while (!(temp == goal) && Dstarmap[temp.x][temp.y].H == Dstarmap[temp.x][temp.y].K);
            if(temp == goal)//if(G_is_reached){ //到达终点
            {
                res.push_back(temp);
                std::cout << "temp == goal" << std::endl;
                return res;//exit;
            }else{
                //Y= State of discrepancy reached trying to move from some State X;
                //DNode candiate = Dstarmap[temp.nextx][temp.nexty];
                //MODIFY-COST(Y,X,newc(Y,X));
                //Dstarmap[candiate.x][candiate.y].H = cost(candiate, temp) + Dstarmap[temp.x][temp.y].H;
                // if(isInList(candiate, closelist))//if( t(Y) == CLOSED )
                // {
                //     insert(candiate, Dstarmap[candiate.x][candiate.y].H);// INSERT( Y, h(Y) );
                // }
                // do
                // {
                //     kmin = processState();//k_min = PROCESS_STATE();   //减少cost和异常信息传播
                // } while (kmin != -1 && kmin < Dstarmap[candiate.x][candiate.y].H);//while( k(Y) < h(Y) && k min != -1);
                    
                // if(kmin == -1)
                // {
                //     std::cout << "else" << std::endl;
                //     return res;//exit;
                // }

                std::vector<DNode> candiates = getNextNodeReplan(temp);
                for(int i = 0; i < candiates.size(); ++i)
                {
                    if(isInList(candiates[i], closelist))//if( t(Y) == CLOSED )
                    {
                        //Dstarmap[candiates[i].x][candiates[i].y].H = Dstarmap[temp.x][temp.y].H + cost(temp, candiates[i]);
                        insert(candiates[i], Dstarmap[candiates[i].x][candiates[i].y].H);// INSERT( Y, h(Y) );
                    }
                    do
                    {
                        kmin = processState();//k_min = PROCESS_STATE();   //减少cost和异常信息传播
                    } while (kmin != -1 && Dstarmap[candiates[i].x][candiates[i].y].K < Dstarmap[candiates[i].x][candiates[i].y].H);//while( k(Y) < h(Y) && k min != -1);
                }
                if(kmin == -1)
                {
                    std::cout << "else" << std::endl;
                    return res;//exit;
                }
            }
        } while (1);//while(1);
    }
    
}

bool DStar::isCanReachGoal(const DNode &a) const
{
    bool flag = false;
    DNode temp = Dstarmap[a.x][a.y];
    //std::cout << "iscanreach first x: " << temp.nextx << ", y: " << temp.nexty << std::endl; 
    while(temp.nextx != -1 && temp.nexty != -1)
    {
        if(map[temp.x][temp.y] == 1)
        {
            flag = false;
            break;
        }
        temp = Dstarmap[temp.nextx][temp.nexty];
        //std::cout << "iscanreach x: " << temp.x << ", y: " << temp.y << std::endl; 
        if(temp == goal)
        {
            flag = true;
            //std::cout << "iscanreach = true" << std::endl;
            break;
        }
    }
    return flag;
}

void DStar::RePlan()
{
    DNode temp = Dstarmap[start.x][start.y];
    int kmin;
    while (!(Dstarmap[temp.nextx][temp.nexty] == goal) && Dstarmap[temp.nextx][temp.nexty].H == Dstarmap[temp.nextx][temp.nexty].K)
    {
        temp = Dstarmap[temp.nextx][temp.nexty];
    }
    if(Dstarmap[temp.nextx][temp.nexty] == goal)
    {
        std::cout << "temp == goal" << std::endl;
        return;
    }else{
        Dstarmap[temp.x][temp.y].H = Dstarmap[temp.nextx][temp.nexty].H + cost(temp, Dstarmap[temp.nextx][temp.nexty]);
        Dstarmap[temp.x][temp.y].K = std::min(Dstarmap[temp.x][temp.y].K, Dstarmap[temp.x][temp.y].H);
        openlist.push_back(Dstarmap[temp.x][temp.y]);
        sortOpenlist();
        if(isInList(temp, closelist))
        {
            deleteInList(temp, closelist);
        }
        do
        {
            if(openlist.empty()) 
            {
                std::cout << "can not find a new path" << std::endl;
                return;
            }
            sortOpenlist();
            DNode mininopen = *openlist.begin();
            openlist.erase(openlist.begin());
            std::cout << "pop (" << mininopen.x << ", " << mininopen.y << "): " << mininopen.K << std::endl;
            std::vector<DNode> candiates = getNextNodeReplan(mininopen);
            std::cout << "candiates size: " << candiates.size() << std::endl;
            for(int i = 0; i < candiates.size(); ++i)
            {
                if(isInList(candiates[i], openlist))
                {
                    int hnew = Dstarmap[candiates[i].x][candiates[i].y].H + cost(candiates[i], mininopen);
                    if(hnew < Dstarmap[mininopen.x][mininopen.y].H)
                    {
                        // deleteInList(candiates[i], openlist);
                        // Dstarmap[candiates[i].x][candiates[i].y].K = std::min(Dstarmap[candiates[i].x][candiates[i].y].H, hnew);
                        // Dstarmap[candiates[i].x][candiates[i].y].H = hnew;
                        // openlist.push_back(Dstarmap[candiates[i].x][candiates[i].y]);
                        Dstarmap[mininopen.x][mininopen.y].H = hnew;
                        Dstarmap[mininopen.x][mininopen.y].K = std::min(Dstarmap[mininopen.x][mininopen.y].K, hnew);
                        if(isCanReachGoal(candiates[i]))
                        {
                            Dstarmap[mininopen.x][mininopen.y].nextx = candiates[i].x;
                            Dstarmap[mininopen.x][mininopen.y].nexty = candiates[i].y;
                            std::cout << "find a new path in openlist" << std::endl;
                            return;
                        }
                    }
                }else if(isInList(candiates[i], closelist)){
                    int hnew = Dstarmap[candiates[i].x][candiates[i].y].H + cost(candiates[i], mininopen);
                    if(hnew < Dstarmap[mininopen.x][mininopen.y].H)
                    {
                        // Dstarmap[candiates[i].x][candiates[i].y].K = std::min(Dstarmap[candiates[i].x][candiates[i].y].H, hnew);
                        // Dstarmap[candiates[i].x][candiates[i].y].H = hnew;
                        Dstarmap[mininopen.x][mininopen.y].H = hnew;
                        Dstarmap[mininopen.x][mininopen.y].K = std::min(Dstarmap[mininopen.x][mininopen.y].K, hnew);
                        deleteInList(candiates[i], closelist);
                        openlist.push_back(Dstarmap[candiates[i].x][candiates[i].y]);
                        if(isCanReachGoal(candiates[i]))
                        {
                            Dstarmap[mininopen.x][mininopen.y].nextx = candiates[i].x;
                            Dstarmap[mininopen.x][mininopen.y].nexty = candiates[i].y;
                            std::cout << "find a new path in closelist" << std::endl;
                            return;
                        }
                    }
                }else{
                    // Dstarmap[candiates[i].x][candiates[i].y].H = Dstarmap[mininopen.x][mininopen.y].H + cost(candiates[i], mininopen);
                    // Dstarmap[candiates[i].x][candiates[i].y].K = Dstarmap[candiates[i].x][candiates[i].y].H;
                    Dstarmap[mininopen.x][mininopen.y].H = Dstarmap[candiates[i].x][candiates[i].y].H + cost(candiates[i], mininopen);
                    Dstarmap[mininopen.x][mininopen.y].K = std::min(Dstarmap[mininopen.x][mininopen.y].K, Dstarmap[mininopen.x][mininopen.y].H);
                    Dstarmap[mininopen.x][mininopen.y].nextx = candiates[i].x;
                    Dstarmap[mininopen.x][mininopen.y].nexty = candiates[i].y;
                    openlist.push_back(Dstarmap[candiates[i].x][candiates[i].y]);
                    std::cout << "a new node" << std::endl;
                }
            }
            closelist.push_back(mininopen);
        }while (1);
    } 
    
}

void DStar::DstarRun(DNode &a, DNode &b)
{
    start = a;
    goal = b;
    Dijstra();
}

std::vector<DNode> DStar::findPath()
{
    std::vector<DNode> res;
    DNode node_ = Dstarmap[start.x][start.y];
    while(node_.nextx != -1 && node_.nexty != -1)
    {
        //std::cout << "(" << node_.x << ", " << node_.y << ") " << std::endl;
        res.push_back(node_);
        node_ = Dstarmap[node_.nextx][node_.nexty];
        if(res.size() == 100) break;//暂用
    }
    //res.push_back(Dstarmap[goal.x][goal.y]);
    //std::cout << "find path" << std::endl;
    return res;
}

DStar::~DStar()
{
}

