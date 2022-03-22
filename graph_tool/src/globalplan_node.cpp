/*
 * @Author: your name
 * @Date: 2022-01-11 15:19:08
 * @LastEditTime: 2022-01-11 16:37:31
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/graph_tool/src/globalplan_node.cpp
 */
#include "graph_tool/globalplan.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_plan_node");
    ros::NodeHandle nh_private("~");

    GlobalPlan plan(nh_private);
    plan.run();
    return 0;
}