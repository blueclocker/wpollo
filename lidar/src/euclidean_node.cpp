/*
 * @Author: your name
 * @Date: 2021-11-14 21:52:22
 * @LastEditTime: 2021-11-14 22:30:13
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/lidar/src/euclidean_node.cpp
 */
#include "lidar/euclidean.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "euclidean");
    ros::NodeHandle nh_ws("~");
    euclideancore test(nh_ws);
    return 0;
}