/*
 * @Author: your name
 * @Date: 2022-03-04 15:30:11
 * @LastEditTime: 2022-11-06 22:18:53
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/main.cpp
 */
#include "navagation/navagation_optimizer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "osmmap");
    ros::NodeHandle nh("~");
    std::string navagation_mode;
    nh.getParam("navagation_mode", navagation_mode);
    navagation::NavagationOptimizer navptr(navagation_mode, nh);
    // map::HDMap bitmap(nh); abondant!
    return 0;
}