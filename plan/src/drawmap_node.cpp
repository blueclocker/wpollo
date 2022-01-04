/*
 * @Author: your name
 * @Date: 2021-11-30 15:06:48
 * @LastEditTime: 2021-12-30 15:43:31
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/src/drawmap_node.cpp
 */
#include "plan/drawmap.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drawmap");
    ros::NodeHandle nh("~");

    std::string map = "/home/wangpeng/fusion/src/plan/maps/";
    GlobalMap mappublish(nh, map, true);
    return 0;
}
