/*
 * @Author: wpbit
 * @Date: 2021-09-27 09:40:22
 * @LastEditTime: 2021-09-27 12:19:39
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fusion/src/test/src/send.cpp
 */

#include"test/send_node.h"

delphi_can::delphi_can(ros::NodeHandle &n)
{
    canInitializeLibrary();
    h = canOpenChannel(0, canWANT_EXCLUSIVE);
    if(h != canOK)
    {
        char msg[64];
        canGetErrorText((canStatus)h, msg, sizeof(msg));
        fprintf(stderr, "canopenchannel failied(%s)\n", msg);
        exit(1);
    }
}

delphi_can::~delphi_can()
{
    canBusOff(h);
    canClose(h);
}


    
void delphi_can::send_canmsg()
{
    canSetBusParams(h, BAUD_500K, 0, 0, 0, 0, 0);
    canBusOn(h);
    long id;
    id = 0x603;
    long int msg;
    msg = 0x2f60600001000000;
    unsigned int dlc;
    dlc = 8;
    unsigned int flag;
    flag = 0;
    canWrite(h, id, &msg, dlc, flag);
    canWriteSync(h, 500);
    ROS_INFO("can0 opened!");
}