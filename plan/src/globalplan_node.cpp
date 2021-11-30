/*
 * @Author: your name
 * @Date: 2021-11-28 20:33:41
 * @LastEditTime: 2021-11-28 23:18:01
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/plan/src/globalplan_node.cpp
 */
#include "plan/globalplan.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "globalplan");
    ros::NodeHandle nh;

    GlobalPlan plan(nh);
    return 0;
}