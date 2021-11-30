/*
 * @Author: your name
 * @Date: 2021-11-18 16:22:09
 * @LastEditTime: 2021-11-18 16:24:53
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /fusion/src/test/src/myfusion.cpp
 */
#include <test/myfusion.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myfusion");
    ros::NodeHandle nh;

    ros::spin();
    return 0;
}