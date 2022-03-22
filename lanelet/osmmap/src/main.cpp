/*
 * @Author: your name
 * @Date: 2022-03-04 15:30:11
 * @LastEditTime: 2022-03-05 19:57:43
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/main.cpp
 */
#include "../include/osmmap/map_io.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "osmmap");
    ros::NodeHandle nh("~");
    map::Map bitmap(nh);
    return 0;
}