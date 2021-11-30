/*
 * @Author: your name
 * @Date: 2021-11-22 13:38:19
 * @LastEditTime: 2021-11-29 15:08:30
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/src/main.cpp
 */
#include "plan/AStar_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    //初始化地图，用二维矩阵代表地图，1表示障碍物，0表示可通
    std::vector<std::vector<int>> map={{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                       {1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1},
                                       {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
                                       {1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1},
                                       {1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1},
                                       {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                                       {1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
                                       {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
    

    Node startpoint(6,10);
    Node goalpoint(1,9);
    clock_t starttime, finishtime;
    starttime = clock();
    AStar_ros aster(&startpoint, &goalpoint, map);//子类对象
    AStar *aster_father = &aster;//父指针
    //AStar *aster_father = new AStar_ros(&startpoint, &goalpoint, map);
    aster_father->solve();//父指针调用子类函数
    finishtime = clock();
    std::cout << "runtime=" << (double)(finishtime - starttime)/CLOCKS_PER_SEC << "s" << std::endl;

    
    ros::spin();
    return 0;
}