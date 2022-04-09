/*
 * @Author: your name
 * @Date: 2022-04-05 14:22:35
 * @LastEditTime: 2022-04-05 14:24:42
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/plan/src/dynamicplannode.cpp
 */
#include "../include/plan/dynamicplan.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamicplan");
    ros::NodeHandle node;
    
    DynamicPlan dynamic(node);
    return 0;
}